//
// Created by Seth Parker on 7/30/15.
//

#include <algorithm>
#include <iostream>
#include <limits>
#include <regex>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include "apps/SliceImage.hpp"
#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/SkyscanMetadataIO.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/VolumePkg.hpp"

using PathStringList = std::vector<std::string>;

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vci = volcart::io;

enum class Flip { None, Horizontal, Vertical, ZFlip, Both };

static const vc::io::ExtensionList AcceptedExtensions{"tif", "tiff", "png",
                                                      "jpg", "jpeg", "bmp"};

static const double MIN_16BPC = std::numeric_limits<uint16_t>::min();
static const double MAX_16BPC = std::numeric_limits<uint16_t>::max();

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;

struct VolumeInfo {
    fs::path path;
    std::string name;
    std::string sliceRegex;
    double voxelsize{0};
    Flip flipOption{Flip::None};
    vc::Metadata meta;
};

VolumeInfo GetVolumeInfo(const fs::path& slicePath);
void AddVolume(vc::VolumePkg::Pointer& volpkg, const VolumeInfo& info);

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description options("Options");
    options.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(),
           "Path for the output volume package")
        ("material-thickness,m", po::value<double>(),
           "Estimated thickness of a material layer (in microns). Required "
           "when making a new volume package.")
        ("slices,s", po::value<PathStringList>(),
            "Path to input slice data. Ends with prefix of slice images or log "
            "file path. Can be specified multiple times to add multiple "
            "volumes.");

    // Useful transforms for origin adjustment
    po::options_description extras("Metadata");
    extras.add_options()
        ("name", po::value<std::string>(),
            "Set a descriptive name for the VolumePkg. Default: Filename "
            "specified by --volpkg");
    // clang-format on
    po::options_description all("Usage");
    all.add(options).add(extras);

    // parsed will hold the values of all parsed options as a Map
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 2) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

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
                      << std::endl;
            return EXIT_FAILURE;
        }
        volpkg = vc::VolumePkg::New(volpkgPath, vc::VOLPKG_VERSION_LATEST);
    } else {
        volpkg = vc::VolumePkg::New(volpkgPath);
    }

    // Make sure we support this version of the VolPkg
    if (volpkg->version() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume package is version " << volpkg->version()
                  << " but this program requires version "
                  << VOLPKG_SUPPORTED_VERSION << "." << std::endl;
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
    PathStringList volumesPaths;
    if (parsed.count("slices")) {
        volumesPaths = parsed["slices"].as<PathStringList>();
    }

    // Get volume info
    std::vector<VolumeInfo> volumesList;
    for (auto& v : volumesPaths) {
        volumesList.emplace_back(GetVolumeInfo(v));
    }

    // Add volumes in sequence
    for (auto& v : volumesList) {
        AddVolume(volpkg, v);
    }
}

VolumeInfo GetVolumeInfo(const fs::path& slicePath)
{
    std::cout << "Getting info for Volume: " << slicePath << std::endl;

    VolumeInfo info;
    bool voxelFound = false;

    // If we have a logfile, try to read it
    if (slicePath.extension().string() == ".log") {
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
    } else {
        info.path = slicePath;
    }

    // Volume Name
    std::cout << "Enter a descriptive name for the volume: ";
    std::getline(std::cin, info.name);

    // Get voxel size
    std::string input;
    if (!voxelFound) {
        do {
            std::cout << "Enter the voxel size of the volume in microns "
                         "(e.g. 13.546): ";
            std::getline(std::cin, input);
        } while (
            !boost::conversion::try_lexical_convert(input, info.voxelsize));
    }

    // Flip options
    std::cout << "Flip options: Vertical flip (vf), horizontal flip (hf), "
                 "both, z-flip (zf), [none] : ";
    std::getline(std::cin, input);

    if (input == "vf") {
        info.flipOption = Flip::Vertical;
    } else if (input == "hf") {
        info.flipOption = Flip::Horizontal;
    } else if (input == "both") {
        info.flipOption = Flip::Both;
    } else if (input == "zf") {
        info.flipOption = Flip::ZFlip;
    }

    return info;
}

void AddVolume(vc::VolumePkg::Pointer& volpkg, const VolumeInfo& info)
{
    std::cout << "Adding Volume: " << info.path << std::endl;

    // Filter the slice path directory by extension and sort the vector of files
    std::cout << "Reading the slice directory..." << std::endl;
    std::vector<volcart::SliceImage> slices;

    if (!fs::exists(info.path) || !fs::is_directory(info.path)) {
        std::cerr << "ERROR: Provided slice path does not exist/is not a "
                     "directory. Please provide a directory of slice images."
                  << std::endl;
        return;
    }

    // Iterate through all files in the directory
    fs::directory_iterator subfile(info.path);
    fs::directory_iterator dirEnd;
    for (; subfile != dirEnd; subfile++) {
        // Skip if not a regular file
        if (!fs::is_regular_file(subfile->path())) {
            continue;
        }

        // Filter by either file extension or the provided regex
        if (info.sliceRegex.empty()) {
            if (vc::io::FileExtensionFilter(
                    subfile->path().filename(), AcceptedExtensions)) {
                slices.emplace_back(subfile->path());
            }
        } else {
            if (std::regex_match(
                    subfile->path().filename().string(),
                    std::regex{info.sliceRegex})) {
                slices.emplace_back(subfile->path());
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
    int32_t counter = 1;
    for (auto& slice : slices) {
        // Report progress
        std::cout << "Analyzing slice: " << counter++ << "/" << slices.size()
                  << "\r" << std::flush;

        // Skip if we can't analyze
        if (!slice.analyze()) {
            continue;
        }

        // Compare all slices to the properties of the first slice
        // Don't quit yet so we can get a list of the problematic files
        if (slice != *slices.begin()) {
            consistent = false;
            std::cerr << std::endl
                      << slice.path.filename()
                      << " does not match the initial slice of the volume."
                      << std::endl;
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
    std::cout << std::endl;

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

    if (info.flipOption == Flip::ZFlip) {
        std::reverse(slices.begin(), slices.end());
    }

    // Do we need to flip?
    auto needsFlip = info.flipOption == Flip::Horizontal ||
                     info.flipOption == Flip::Vertical ||
                     info.flipOption == Flip::Both;

    // Move the slices into the VolPkg
    counter = 0;
    for (auto& slice : slices) {
        std::cout << "Saving slice image to volume package: " << counter + 1
                  << "/" << slices.size() << "\r" << std::flush;

        // Convert or flip
        if (slice.needsConvert() || slice.needsScale() || needsFlip) {
            // Override slice min/max with volume min/max
            if (slice.needsScale()) {
                slice.setScale(volMax, volMin);
            }

            // Get slice
            auto tmp = slice.conformedImage();

            // Apply flips
            switch (info.flipOption) {
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
                    // Do nothing
                    break;
                case Flip::None:
                    // Do nothing
                    break;
            }

            // Add to volume
            volume->setSliceData(counter, tmp);
        }

        // Just copy to the volume
        else {
            fs::copy_file(slice.path, volume->getSlicePath(counter));
        }

        ++counter;
    }
    std::cout << std::endl;
}
