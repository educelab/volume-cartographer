#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <regex>
#include <vector>

#include <boost/program_options.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/apps/packager/SliceImage.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/FileFilters.hpp"
#include "vc/core/io/SkyscanMetadataIO.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/FormatStrToRegexStr.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/String.hpp"

using StringList = std::vector<std::string>;
using DoubleList = std::vector<double>;

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vci = volcart::io;

enum class Flip { None, Horizontal, Vertical, ZFlip, Both, All };

static const vci::ExtensionList ImageExts{"tif", "tiff", "png",
                                          "jpg", "jpeg", "bmp"};

static const double MIN_16BPC = std::numeric_limits<std::uint16_t>::min();
static const double MAX_16BPC = std::numeric_limits<std::uint16_t>::max();

// Volpkg version required by this app
static constexpr int VOLPKG_MIN_VERSION = 6;

struct VolumeInfo {
    fs::path path;
    std::string name;
    std::string sliceRegex;
    double voxelsize{0};
    Flip flipOption{Flip::None};
    vc::Metadata meta;
    bool compress{false};
    bool forceWrite{false};
};

static bool DoAnalyze{true};

auto ExtractVolumeOptions(
    po::parsed_options& parsed, const po::options_description& volOptDesc)
    -> std::vector<po::parsed_options>;
auto GetVolumeInfo(const po::variables_map& parsed) -> VolumeInfo;
void AddVolume(vc::VolumePkg::Pointer& volpkg, const VolumeInfo& info);

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description options("Options");
    options.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(),
           "Path for the output volume package.");

    po::options_description volpkg_metadata("Volpkg metadata");
    volpkg_metadata.add_options()
        ("name", po::value<std::string>(),
            "Set a descriptive name for the VolumePkg. Default: Filename "
            "specified by --volpkg")
        ("material-thickness,m", po::value<double>(),
           "Estimated thickness of a material layer (in microns). Required "
           "when making a new volume package.");

    po::options_description volume_options("Volume");
    volume_options.add_options()
        ("slices,s", po::value<StringList>(),
            "Path to input slice data. Ends with prefix of slice images or log "
            "file path. Required when making a new volume. If specified "
            "multiple times, volume options will be associated with the "
            "previous slices.")
        ("volume-name,n", po::value<StringList>(),
            "Descriptive name for the volume. Required when making a new "
            "volume.")
        ("voxel-size-um,u", po::value<DoubleList>(),
            "Voxel size of the volume in microns (e.g. 13.546). Required when "
            "making a new volume.")
        ("flip,f", po::value<StringList>(),
            "Flip options: Vertical flip (vf), horizontal flip (hf), both, "
            "z-flip (zf), all, [none].")
        ("compress,c", "Compress slice images");
    
    po::options_description helpOpts("Usage");
    helpOpts.add(options).add(volpkg_metadata).add(volume_options);

    po::options_description all("Usage");
    all.add(helpOpts).add_options()(
        "analyze", po::value<bool>()->default_value(true), "Analyze volume")
        ("force-write", "Force writing new TIFF files even if they could be copied");
    // clang-format on

    // Parse the command line and separate out flags for volumes
    auto parsed = po::command_line_parser(argc, argv).options(all).run();
    auto parsedVols = ExtractVolumeOptions(parsed, volume_options);

    // args will hold the values of the parsed main options as a Map
    po::variables_map args;
    po::store(parsed, args);

    // vargs will hold a list of grouped volume options, one for each volume
    std::vector<po::variables_map> vargs;
    for (const auto& p : parsedVols) {
        po::variables_map arg;
        po::store(p, arg);
        vargs.push_back(arg);
    }

    // Show the help message
    if (args.count("help") || argc < 2) {
        std::cout << helpOpts << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(args);
        for (auto& arg : vargs) {
            po::notify(arg);
        }
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // Set global opt
    DoAnalyze = args["analyze"].as<bool>();

    ///// New VolumePkg /////
    // Get the output volpkg path
    fs::path volpkgPath = args["volpkg"].as<std::string>();
    auto newPackageMode = !fs::exists(volpkgPath);

    // Make sure the package doesn't already exist
    vc::VolumePkg::Pointer volpkg;
    if (newPackageMode) {
        // Check the extension
        if (volpkgPath.extension().string() != ".volpkg") {
            volpkgPath.replace_extension(".volpkg");
        }
        // Make sure we have the material thickness value
        if (args.count("material-thickness") == 0) {
            std::cerr << "ERROR: Making a new volume package but did not "
                         "provide the material thickness."
                      << '\n';
            return EXIT_FAILURE;
        }
        volpkg = vc::VolumePkg::New(volpkgPath, vc::VOLPKG_VERSION_LATEST);
    } else {
        volpkg = vc::VolumePkg::New(volpkgPath);
    }

    // Make sure we support this version of the VolPkg
    if (volpkg->version() < VOLPKG_MIN_VERSION) {
        vc::Logger()->error(
            "Volume Package is version {} but this program requires version "
            "{}+. ",
            volpkg->version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    // Get volpkg name
    std::string vpkgName;
    if (args.count("name")) {
        vpkgName = args["name"].as<std::string>();
    } else if (newPackageMode) {
        vpkgName = volpkgPath.stem().string();
    }
    if (!vpkgName.empty()) {
        volpkg->setMetadata("name", vpkgName);
    }

    // Get material thickness
    if (args.count("material-thickness")) {
        auto thickness = args["material-thickness"].as<double>();
        volpkg->setMetadata("materialthickness", thickness);
    }

    // Update metadata on disk
    volpkg->saveMetadata();

    ///// Add Volumes /////
    for (const auto& v : vargs) {
        VolumeInfo info = GetVolumeInfo(v);
        info.forceWrite = args.count("force-write") > 0;
        AddVolume(volpkg, info);
    }
}

// Extract volume-related options into a separate list
auto ExtractVolumeOptions(
    po::parsed_options& parsed, const po::options_description& volOptDesc)
    -> std::vector<po::parsed_options>
{
    // partition options between volume-related and others
    po::parsed_options parsedVolOpts(&volOptDesc);
    po::parsed_options parsedOtherOpts(parsed.description);
    for (auto& o : parsed.options) {
        if (volOptDesc.find_nothrow(o.string_key, false) != nullptr) {
            parsedVolOpts.options.push_back(o);
        } else {
            parsedOtherOpts.options.push_back(o);
        }
    }
    // replace "parsed" with our extracted "other" opts
    parsed = std::move(parsedOtherOpts);

    // iterate over list of options and split every time we see --slices
    std::vector<po::parsed_options> volOpts;
    std::optional<po::parsed_options> volOpt;
    for (const auto& p : parsedVolOpts.options) {
        // start a new volOpt when we encounter --slices
        if (p.string_key == "slices") {
            // add previous volOpt to output list as needed
            if (volOpt.has_value()) {
                volOpts.push_back(volOpt.value());
            }
            volOpt = po::parsed_options(&volOptDesc);
        }

        // if we've started a volOpt, add this option to it
        if (volOpt.has_value()) {
            volOpt.value().options.push_back(p);
        }
    }
    // Add the final volOpt to our output list
    if (volOpt.has_value()) {
        volOpts.push_back(volOpt.value());
    }

    return volOpts;
}

auto GetVolumeInfo(const po::variables_map& parsed) -> VolumeInfo
{
    VolumeInfo info;

    const fs::path slicePath = parsed["slices"].as<StringList>().front();

    bool voxelFound = false;

    // If path is a log file, try to read its info
    if (vci::FileExtensionFilter(slicePath.filename(), {"log"})) {
        // Get the parent directory for the log
        info.path = slicePath.parent_path();

        // Read the Skyscan metadata
        vc::SkyscanMetadataIO logReader;
        logReader.setPath(slicePath);
        info.meta = logReader.read();

        // Get the slice file regex
        info.sliceRegex = logReader.getSliceRegexString();

        // Try to get the voxel size
        if (info.meta.hasKey("voxelSize")) {
            info.voxelsize = info.meta.get<double>("voxelSize").value();
            voxelFound = true;
        } else {
            std::cerr << "Warning: Log file does not contain voxel size. Is "
                         "this a reconstruction log?\n";
        }
    }

    // If the path is an image format, assume its a printf-pattern
    else if (vci::FileExtensionFilter(slicePath.filename(), ImageExts)) {
        info.path = slicePath.parent_path();
        info.sliceRegex =
            vc::FormatStrToRegexStr(slicePath.filename().string());
    }

    // Otherwise, assume it's a directory containing only slice images
    else {
        info.path = slicePath;
    }

    // Volume Name
    if (parsed.count("volume-name") == 0) {
        std::cerr << "ERROR: --volume-name required when creating a new volume."
                  << '\n';
        exit(EXIT_FAILURE);
    }
    info.name = parsed["volume-name"].as<StringList>().back();

    // Get voxel size
    if (!voxelFound) {
        if (parsed.count("voxel-size-um") == 0) {
            std::cerr
                << "ERROR: --voxel-size-um required when creating a new volume."
                << '\n';
            exit(EXIT_FAILURE);
        }
        info.voxelsize = parsed["voxel-size-um"].as<DoubleList>().back();
    }

    // Flip options
    if (parsed.count("flip") != 0) {
        auto flip = parsed["flip"].as<StringList>().back();
        vc::to_lower(flip);

        if (flip == "vf") {
            info.flipOption = Flip::Vertical;
        } else if (flip == "hf") {
            info.flipOption = Flip::Horizontal;
        } else if (flip == "both") {
            info.flipOption = Flip::Both;
        } else if (flip == "zf") {
            info.flipOption = Flip::ZFlip;
        } else if (flip == "all") {
            info.flipOption = Flip::All;
        } else if (flip == "none") {
            info.flipOption = Flip::None;
        } else if (not flip.empty()) {
            std::cerr << "Ignoring unrecognized flip option: " << flip << "\n";
        }
    }

    // Whether to compress
    info.compress = parsed.count("compress") != 0;

    return info;
}

void AddVolume(vc::VolumePkg::Pointer& volpkg, const VolumeInfo& info)
{
    std::cout << "Adding Volume: " << info.path << std::endl;

    // Filter the slice path directory by extension and sort the vector of files
    std::cout << "Reading the slice directory..." << std::endl;
    std::vector<vc::SliceImage> slices;

    if (not fs::exists(info.path) or not fs::is_directory(info.path)) {
        std::cerr << "ERROR: Provided slice path does not exist/is not a "
                     "directory. Please provide a directory of slice images."
                  << std::endl;
        return;
    }

    // Iterate through all files in the directory
    fs::directory_iterator subfile(info.path);
    for (const fs::directory_iterator dirEnd; subfile != dirEnd; ++subfile) {
        // Get subfile as path
        const auto subpath = subfile->path();

        // Skip if not a regular or visible file
        if (not fs::is_regular_file(subpath) or vc::IsUnixHiddenFile(subpath)) {
            continue;
        }

        // Filter by either file extension or the provided regex
        if (info.sliceRegex.empty()) {
            if (vc::IsFileType(subpath, ImageExts)) {
                slices.emplace_back(subpath);
            }
        } else {
            if (std::regex_match(
                    subpath.filename().string(), std::regex{info.sliceRegex})) {
                slices.emplace_back(subpath);
            }
        }
    }

    // Return if we didn't find any slices
    if (slices.empty()) {
        std::cerr << "ERROR: No supported image files found in provided slices "
                     "directory."
                  << std::endl;
        return;
    }

    // Sort the Slices by their filenames
    std::sort(slices.begin(), slices.end());

    // Report the number of slices
    std::cout << "Slice images found: " << slices.size() << std::endl;

    ///// Analyze the slices /////
    auto consistent = true;
    auto volMin = std::numeric_limits<double>::max();
    auto volMax = std::numeric_limits<double>::lowest();
    std::vector<fs::path> mismatches;
    if (DoAnalyze) {
        for (auto& slice : vc::ProgressWrap(slices, "Analyzing slices")) {
            // Skip if we can't analyze
            if (!slice.analyze()) {
                continue;
            }

            // Compare all slices to the properties of the first slice
            // Don't quit yet so we can get a list of the problematic files
            if (slice != *slices.begin()) {
                consistent = false;
                mismatches.push_back(slice.path.filename());
                continue;
            }

            // Update the volume's min and max
            if (slice.min() < volMin) {
                volMin = slice.min();
            }

            if (slice.max() > volMax) {
                volMax = slice.max();
            }
        }
    } else {
        // Make sure the first slice is analyzed
        slices.front().analyze();
        volMin = MIN_16BPC;
        volMax = MAX_16BPC;
    }

    // Report mismatched slices
    if (not mismatches.empty()) {
        std::cerr << "Found " << mismatches.size();
        std::cerr << " files which did not match the initial slice:";
        std::cerr << std::endl;
        for (const auto& p : mismatches) {
            std::cerr << "\t" << p << std::endl;
        }
    }

    // Quit if the volume isn't consistent
    if (!consistent) {
        std::cerr << "ERROR: Slices in slice directory do not have matching "
                     "properties (width/height/depth)."
                  << std::endl;
        return;
    }

    ///// Add data to the volume /////
    // Metadata
    auto volume = volpkg->newVolume(info.name);
    volume->setNumberOfSlices(slices.size());
    volume->setSliceWidth(slices.front().width());
    volume->setSliceHeight(slices.front().height());
    volume->setVoxelSize(info.voxelsize);

    // Scale min/max values
    if (slices.begin()->needsScale()) {
        volume->setMin(MIN_16BPC);
        volume->setMax(MAX_16BPC);
    } else {
        volume->setMin(volMin);
        volume->setMax(volMax);
    }
    volume->saveMetadata();

    if (info.flipOption == Flip::ZFlip or info.flipOption == Flip::All) {
        std::reverse(slices.begin(), slices.end());
    }

    // Do we need to flip?
    auto needsFlip = info.flipOption == Flip::Horizontal ||
                     info.flipOption == Flip::Vertical ||
                     info.flipOption == Flip::Both ||
                     info.flipOption == Flip::All;

    // Move the slices into the VolPkg
    using vc::enumerate;
    using vc::ProgressWrap;
    for (auto pair : ProgressWrap(enumerate(slices), "Saving to volpkg")) {
        const auto& idx = pair.first;
        auto& slice = pair.second;
        // Convert or flip
        if (slice.needsConvert() || slice.needsScale() || needsFlip ||
            info.compress || info.forceWrite) {
            // Override slice min/max with volume min/max
            if (slice.needsScale()) {
                slice.setScale(volMax, volMin);
            }

            // Get slice
            auto tmp = slice.conformedImage();

            // Apply flips
            switch (info.flipOption) {
                case Flip::All:
                case Flip::Both:
                    cv::flip(tmp, tmp, -1);
                    break;
                case Flip::Vertical:
                    cv::flip(tmp, tmp, 0);
                    break;
                case Flip::Horizontal:
                    cv::flip(tmp, tmp, 1);
                    break;
                case Flip::ZFlip:
                case Flip::None:
                    // Do nothing
                    break;
            }

            // Add to volume
            volume->setSliceData(idx, tmp, info.compress);
        }

        // Just copy to the volume
        else {
            fs::copy_file(slice.path, volume->getSlicePath(idx));
        }
    }
}
