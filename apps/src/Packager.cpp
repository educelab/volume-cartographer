//
// Created by Seth Parker on 7/30/15.
//

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include "apps/SliceImage.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char* argv[])
{

    ///// Parse the command line options /////
    fs::path slicesPath, volpkgPath;
    bool verticalFlip, horizontalFlip;
    try {
        // All command line options
        // clang-format off
        po::options_description options("Options");
        options.add_options()
            ("help,h", "Show this message")
            ("slices,s", po::value<std::string>(),
                "Directory of input slice data")
            ("volpkg,v", po::value<std::string>(),
                "Path for the output volume package");

        // Useful transforms for origin adjustment
        po::options_description extras("Volume Transformations");
        extras.add_options()
            ("horizontal-flip", po::bool_switch()->default_value(false),
             "Apply a horizontal flip to slice images.")
            ("vertical-flip", po::bool_switch()->default_value(false),
             "Apply a vertical flip to slice images.");
        // clang-format on
        po::options_description all("Usage");
        all.add(options).add(extras);

        // parsedOptions will hold the values of all parsed options as a Map
        po::variables_map parsedOptions;
        po::store(
            po::command_line_parser(argc, argv).options(all).run(),
            parsedOptions);
        po::notify(parsedOptions);

        // Show the help message
        if (parsedOptions.count("help") || argc < 2) {
            std::cout << all << std::endl;
            return EXIT_SUCCESS;
        }

        // Get the input slices dir
        if (parsedOptions.count("slices")) {
            slicesPath = parsedOptions["slices"].as<std::string>();
        } else {
            std::cerr << "ERROR: Path to slices directory not supplied!"
                      << std::endl;
            std::cout << options << std::endl;
            return EXIT_FAILURE;
        }

        // Get the output volpkg path
        if (parsedOptions.count("volpkg")) {
            volpkgPath = parsedOptions["volpkg"].as<std::string>();
        } else {
            std::cerr << "ERROR: Output Volume Package path not supplied!"
                      << std::endl;
            std::cout << options << std::endl;
            return EXIT_FAILURE;
        }

        // Flips?
        verticalFlip = parsedOptions["vertical-flip"].as<bool>();
        horizontalFlip = parsedOptions["horizontal-flip"].as<bool>();
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    ///// Get metadata values that have to be manually set /////
    std::string input;
    double voxelsize, thickness;

    // get voxel size
    do {
        std::cout << "Please enter the voxel size of the volume in microns "
                     "(e.g. 13.546): ";
        std::getline(std::cin, input);
    } while (!boost::conversion::try_lexical_convert(input, voxelsize));
    // get material thickness
    do {
        std::cout << "Please enter the estimated material thickness of the "
                     "volume in microns (e.g. 56.026): ";
        std::getline(std::cin, input);
    } while (!boost::conversion::try_lexical_convert(input, thickness));

    ///// Setup /////
    // Check the extension and make sure the pkg doesn't already exist
    if (volpkgPath.extension().string() != ".volpkg")
        volpkgPath.replace_extension(".volpkg");
    if (fs::exists(volpkgPath)) {
        std::cerr << "ERROR: Volume package already exists at path specified."
                  << std::endl;
        std::cerr << "This program does not currently allow for modification "
                     "of existing volume packages."
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Generate an empty volpkg and save it to disk
    VolumePkg volpkg(volpkgPath.string(), VOLPKG_VERSION_LATEST);
    volpkg.readOnly(false);
    volpkg.setMetadata("volumepkg name", volpkgPath.stem().string());
    volpkg.setMetadata("voxelsize", voxelsize);
    volpkg.setMetadata("materialthickness", thickness);

    // Filter the slice path directory by extension and sort the vector of files
    std::cout << "Reading the slice directory..." << std::endl;
    std::vector<volcart::SliceImage> slices;
    if (fs::exists(slicesPath) && fs::is_directory(slicesPath)) {

        // Directory iterators
        fs::directory_iterator dir_subfile(slicesPath);
        fs::directory_iterator dir_end;

        // Filter out subfiles that aren't TIFs
        // To-Do: #177
        while (dir_subfile != dir_end) {
            std::string file_ext(boost::to_upper_copy<std::string>(
                dir_subfile->path().extension().string()));
            if (is_regular_file(dir_subfile->path()) &&
                (file_ext == ".TIF" || file_ext == ".TIFF")) {
                volcart::SliceImage temp;
                temp.path = *dir_subfile;
                slices.push_back(temp);
            }
            ++dir_subfile;
        }
    } else {
        std::cerr
            << "ERROR: Slices directory does not exist/is not a directory."
            << std::endl;
        std::cerr << "Please provide a directory of slice images." << std::endl;
        return EXIT_FAILURE;
    }
    if (slices.empty()) {
        std::cerr << "ERROR: No supported image files found in provided slices "
                     "directory."
                  << std::endl;
        return EXIT_FAILURE;
    }
    // Sort the Slices by their filenames
    std::sort(slices.begin(), slices.end(), SlicePathLessThan);
    std::cout << "Slice images found: " << slices.size() << std::endl;
    volpkg.volume().setNumberOfSlices(slices.size());

    ///// Analyze the slices /////
    bool vol_consistent = true;
    double vol_min{}, vol_max{};
    uint64_t counter = 1;
    for (auto slice = slices.begin(); slice != slices.end(); ++slice) {
        std::cout << "Analyzing slice: " << counter << "/" << slices.size()
                  << "\r" << std::flush;
        if (!slice->analyze())
            continue;  // skip if we can't analyze

        // Compare all slices to the properties of the first slice
        if (slice == slices.begin()) {
            vol_min = slice->min();
            vol_max = slice->max();
        } else {
            // Check for consistency of slices
            if (*slice != *slices.begin()) {
                vol_consistent = false;
                std::cerr << std::endl
                          << slice->path.filename()
                          << " does not match the initial slice of the volume."
                          << std::endl;
                continue;
            }

            // Update the volume's min and max
            if (slice->min() < vol_min)
                vol_min = slice->min();
            if (slice->max() > vol_max)
                vol_max = slice->max();
        }

        ++counter;
    }
    std::cout << std::endl;
    if (!vol_consistent) {
        std::cerr << "ERROR: Slices in slice directory do not have matching "
                     "properties (width/height/depth)."
                  << std::endl;
        return EXIT_FAILURE;
    }

    ///// Add data to the volume package /////
    // Metadata
    volpkg.setMetadata("number of slices", slices.size());
    volpkg.setMetadata("width", slices.begin()->width());
    volpkg.setMetadata("height", slices.begin()->height());

    // Scale 8-bit min/max values
    // To-Do: Handle other bit depths
    if (slices.begin()->depth() == 0) {
        vol_min = vol_min * 65535.00 / 255.00;
        vol_max = vol_max * 65535.00 / 255.00;
    }
    volpkg.setMetadata("min", vol_min);
    volpkg.setMetadata("max", vol_max);
    volpkg.saveMetadata();  // Save final metadata changes to disk

    counter = 0;
    for (auto slice = slices.begin(); slice != slices.end(); ++slice) {
        std::cout << "Saving slice image to volume package: " << counter + 1
                  << "/" << slices.size() << "\r" << std::flush;
        if (slice->needsConvert() || verticalFlip || horizontalFlip) {
            // Get slice
            auto tmp = slice->conformedImage();

            // Apply flips
            if (verticalFlip && horizontalFlip) {
                cv::flip(tmp, tmp, -1);
            } else if (verticalFlip) {
                cv::flip(tmp, tmp, 0);
            } else if (horizontalFlip) {
                cv::flip(tmp, tmp, 1);
            }

            // Add to volume
            volpkg.setSliceData(counter, tmp);
        } else {
            fs::copy(slice->path, volpkg.volume().getSlicePath(counter));
        }

        ++counter;
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
