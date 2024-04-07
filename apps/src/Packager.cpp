#include <algorithm>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <limits>
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
};

static bool DoAnalyze{true};

void ExtractVolumeOptions(
    po::parsed_options& command_line,
    std::vector<po::parsed_options>& volumes_command_line,
    const po::options_description& volume_options);
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
        "analyze", po::value<bool>()->default_value(true), "Analyze volume");
    // clang-format on

    po::parsed_options command_line =
        po::command_line_parser(argc, argv).options(all).run();
    std::vector<po::parsed_options> volumes_command_line;
    ExtractVolumeOptions(command_line, volumes_command_line, volume_options);

    // parsed will hold the values of the parsed main options as a Map
    po::variables_map parsed;
    po::store(command_line, parsed);

    // parsed_volumes will hold the values of the parsed volumes options
    std::vector<po::variables_map> parsed_volumes;
    for (const auto& volume_command_line : volumes_command_line) {
        po::variables_map parsed_volume;
        po::store(volume_command_line, parsed_volume);
        parsed_volumes.push_back(parsed_volume);
    }

    // Show the help message
    if (parsed.count("help") || argc < 2) {
        std::cout << helpOpts << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
        for (auto& parsed_volume : parsed_volumes) {
            po::notify(parsed_volume);
        }
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // Set global opt
    DoAnalyze = parsed["analyze"].as<bool>();

    ///// New VolumePkg /////
    // Get the output volpkg path
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    auto newPackageMode = !fs::exists(volpkgPath);

    // Make sure the package doesn't already exist
    vc::VolumePkg::Pointer volpkg;
    if (newPackageMode) {
        // Check the extension
        if (volpkgPath.extension().string() != ".volpkg") {
            volpkgPath.replace_extension(".volpkg");
        }
        // Make sure we have the material thickness value
        if (parsed.count("material-thickness") == 0) {
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
    if (parsed.count("name")) {
        vpkgName = parsed["name"].as<std::string>();
    } else if (newPackageMode) {
        vpkgName = volpkgPath.stem().string();
    }
    if (!vpkgName.empty()) {
        volpkg->setMetadata("name", vpkgName);
    }

    // Get material thickness
    if (parsed.count("material-thickness")) {
        auto thickness = parsed["material-thickness"].as<double>();
        volpkg->setMetadata("materialthickness", thickness);
    }

    // Update metadata on disk
    volpkg->saveMetadata();

    ///// Add Volumes /////
    for (const auto& parsed_volume : parsed_volumes) {
        VolumeInfo info = GetVolumeInfo(parsed_volume);
        AddVolume(volpkg, info);
    }
}

void ExtractVolumeOptions(
    po::parsed_options& command_line,
    std::vector<po::parsed_options>& volumes_command_line,
    const po::options_description& volume_options)
{
    // partition options between volumes and others
    po::parsed_options all_volumes_command_line(&volume_options);
    po::parsed_options others_command_line(command_line.description);

    for (auto& o : command_line.options) {
        if (volume_options.find_nothrow(o.string_key, false) != nullptr) {
            all_volumes_command_line.options.push_back(o);
        } else {
            others_command_line.options.push_back(o);
        }
    }

    command_line = std::move(others_command_line);

    // get iterators to "slices" options
    std::vector<std::vector<po::option>::const_iterator> slices_iterators;

    for (auto it = all_volumes_command_line.options.cbegin();
         it != all_volumes_command_line.options.cend(); it++) {
        if (it->string_key == "slices") {
            slices_iterators.push_back(it);
        }
    }

    if (slices_iterators.size() == 1) {
        // only one volume with all options
        volumes_command_line.push_back(std::move(all_volumes_command_line));
        return;
    }

    // group individual volumes options together
    slices_iterators.push_back(all_volumes_command_line.options.cend());

    for (auto it = slices_iterators.cbegin();
         it != (slices_iterators.cend() - 1); it++) {
        po::parsed_options volume_command_line(&volume_options);
        std::copy(
            *it, *(it + 1), std::back_inserter(volume_command_line.options));
        volumes_command_line.push_back(std::move(volume_command_line));
    }
}

auto GetVolumeInfo(const po::variables_map& parsed) -> VolumeInfo
{
    VolumeInfo info;

    fs::path slicePath = parsed["slices"].as<StringList>().front();

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
        try {
            info.voxelsize = info.meta.get<double>("voxelSize");
            voxelFound = true;
        } catch (const std::exception&) {
            std::cerr << "Warning: Log file does not contain voxel size. Is "
                         "this a reconstruction log?"
                      << std::endl;
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
            info.compress) {
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
