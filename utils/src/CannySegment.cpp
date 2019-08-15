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

// Canny values
int blurSlider = 1;
int gaussianKernel = 3;
int minVal = 0;
int maxVal = 255;
int apertureSlider = 0;
int aperture = 3;

// Show Canny opts
std::string windowName = "Canny Output";
cv::Mat src, dst, mask;
void ShowCanny(const cv::Mat& i);
void CannyThreshold(int, void*);

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
        ("calculate-midpoint", "When enabled, try to find a surface midpoint")
        ("threshold-min", po::value<int>()->default_value(100), "Minimum Intensity Gradient")
        ("threshold-max", po::value<int>()->default_value(150), "Maximum Intensity Gradient")
        ("visualize", "Show Canny visualization before segmenting")
        ("slice", po::value<int>()->default_value(0), "Slice to visualize")
        ("mask", po::value<std::string>(), "Mask the output of Canny using the provided image");

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
    minVal = parsed["threshold-min"].as<int>();
    maxVal = parsed["threshold-max"].as<int>();

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

    if (parsed.count("mask") > 0) {
        mask = cv::imread(parsed["mask"].as<std::string>(), CV_8UC1);
    }

    if (parsed.count("visualize") > 0) {
        ShowCanny(vc::QuantizeImage(volume->getSliceDataCopy(parsed["slice"].as<int>()), CV_8UC1));
    }

    auto midpoint = parsed.count("calculate-midpoint") > 0;

    // Segment
    std::cout << "Segmenting surface..." << std::endl;
    auto mesh = vc::ITKMesh::New();
    cv::Vec3d first, last, middle;
    for (int z = 0; z < volume->numSlices(); z++) {
        // Get the slice and blur it
        auto slice = vc::QuantizeImage(volume->getSliceDataCopy(z), CV_8UC1);
        cv::GaussianBlur(slice, slice, {gaussianKernel, gaussianKernel}, 0);

        // Run Canny Edge Detect
        cv::Mat cannySlice;
        cv::Canny(slice, cannySlice, minVal, maxVal, aperture, true);

        cv::Mat processed;
        cannySlice.copyTo(processed, mask);

        // Convert first canny edge along y axis to surface point
        for (int x = 0; x < processed.cols; x++) {
            // Get the first
            auto haveFirst = false;
            for (int y = 0; y < processed.rows; y++) {
                if (!haveFirst && processed.at<uint8_t>(y, x) != 0) {
                    first = {x, y, z};
                    last = first;
                    haveFirst = true;
                    continue;
                }

                if (midpoint && haveFirst && processed.at<uint8_t>(y, x) != 0) {
                    last = {x, y, z};
                }

                if (!midpoint && haveFirst) {
                    break;
                }
            }

            if (!haveFirst) {
                continue;
            }

            middle = (first + last) / 2;
            mesh->SetPoint(mesh->GetNumberOfPoints(), middle.val);
        }
    }

    // Write mesh
    std::cout << "Writing mesh..." << std::endl;
    vc::io::PLYWriter writer;
    writer.setMesh(mesh);
    writer.setPath(outputPath);
    writer.write();
}

void ShowCanny(const cv::Mat& i)
{
    src = i;
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::startWindowThread();
    cv::createTrackbar(
        "Blur Size:", windowName, &blurSlider, 30, CannyThreshold);
    cv::createTrackbar(
        "Min Threshold:", windowName, &minVal, 255, CannyThreshold);
    cv::createTrackbar(
        "Max Threshold:", windowName, &maxVal, 255, CannyThreshold);
    cv::createTrackbar(
        "Aperture Size:", windowName, &apertureSlider, 2, CannyThreshold);

    CannyThreshold(0, nullptr);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void CannyThreshold(int, void*)
{
    gaussianKernel = 2 * blurSlider + 1;
    aperture = 2 * apertureSlider + 3;
    cv::Mat canny;
    cv::GaussianBlur(src, canny, {gaussianKernel, gaussianKernel}, 0);

    // Run Canny Edge Detect
    cv::Canny(canny, canny, minVal, maxVal, aperture, true);

    dst = cv::Scalar::all(0);
    canny.copyTo(dst, mask);

    cv::addWeighted(src, 0.5, dst, 0.5, 0, dst);

    cv::imshow(windowName, dst);
}
