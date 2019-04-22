#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/QuantizeImage.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("volume", po::value<std::string>(),
           "Volume to use for segmentation. Default: First volume")
        ("output-file,o", po::value<std::string>()->required(),
           "Output mesh path (PLY)");

    po::options_description segOpts("Segmentation Options");
    segOpts.add_options()
        ("threshold", po::value<int>()->default_value(100), "Canny Threshold #1")
        ("threshold2", po::value<int>()->default_value(150), "Canny Threshold #2");

    po::options_description all("Usage");
    all.add(required).add(segOpts);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") != 0 || argc < 4) {
        std::cerr << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get options
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    fs::path outputPath = parsed["output-file"].as<std::string>();
    auto threshold = parsed["threshold"].as<int>();
    auto threshold2 = parsed["threshold2"].as<int>();

    ///// Load the VolumePkg /////
    auto vpkg = vc::VolumePkg::New(volpkgPath);

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    try {
        if (parsed.count("volume") != 0) {
            volume = vpkg->volume(parsed["volume"].as<std::string>());
        } else {
            volume = vpkg->volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct."
                  << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Segment
    std::cout << "Segmenting surface..." << std::endl;
    auto mesh = vc::ITKMesh::New();
    vc::ITKPoint pt;
    for (int z = 0; z < volume->numSlices(); z++) {
        // Get the slice and blur it
        auto slice = vc::QuantizeImage(volume->getSliceDataCopy(z), CV_8UC1);
        cv::GaussianBlur(slice, slice, {3, 3}, 0);

        // Run Canny Edge Detect
        cv::Mat cannySlice;
        cv::Canny(slice, cannySlice, threshold, threshold2, 3, true);

        // Convert first canny edge along y axis to surface point
        for (int x = 0; x < cannySlice.cols; x++) {
            for (int y = 0; y < cannySlice.rows; y++) {
                if (cannySlice.at<uint8_t>(y, x) != 0) {
                    pt[0] = x;
                    pt[1] = y;
                    pt[2] = z;
                    mesh->SetPoint(mesh->GetNumberOfPoints(), pt);
                    break;
                }
            }
        }
    }

    // Write mesh
    std::cout << "Writing mesh..." << std::endl;
    vc::io::PLYWriter writer;
    writer.setMesh(mesh);
    writer.setPath(outputPath);
    writer.write();
}
