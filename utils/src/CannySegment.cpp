#include <iostream>
#include <locale>
#include <vector>

#include <boost/program_options.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/ImageConversion.hpp"
#include "vc/core/util/Iteration.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// using vc::ProgressWrap;
using vc::range;
using vc::range2D;

// Canny values
int blurSlider = 1;
int gaussianKernel = 3;
int closingSlider = 4;
int closingKernel = 9;
int minVal = 0;
int maxVal = 255;
int apertureSlider = 0;
int aperture = 3;
int sliceIdx = 0;
bool contour{false};
bool bilateral{false};

vc::Volume::Pointer volume;

// Show Canny opts
std::string outputWindowName = "Canny Output";
std::string settingsWindowName = "Canny Settings";
cv::Mat src, dst, mask;
void ShowCanny();
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
        ("mask", po::value<std::string>(), "Mask the output of Canny using the provided image")
        ("use-contour,c", "If enabled, draw contour around canny edges before projection")
        ("bilateral,b", "If enabled, bilateral filter image")
        ("projection-edge,e", po::value<std::string>()->default_value("L"), "Edge to segment from: (L)eft, (R)ight, (T)op, (B)ottom, (N)one");

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
    contour = parsed.count("use-contour") > 0;
    bilateral = parsed.count("bilateral") > 0;

    auto edge = parsed["projection-edge"].as<std::string>();
    std::transform(edge.begin(), edge.end(), edge.begin(), ::toupper);
    if (!(edge == "L" || edge == "R" || edge == "T" || edge == "B" ||
          edge == "N")) {
        std::cerr << "ERROR: "
                  << "projection-edge must be one of L,R,T,B,N" << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the VolumePkg /////
    auto vpkg = vc::VolumePkg::New(volpkgPath);

    ///// Load the Volume /////
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
        ShowCanny();
    }

    auto midpoint = parsed.count("calculate-midpoint") > 0;

    // Segment
    std::cout << "Segmenting surface..." << std::endl;
    auto mesh = vc::ITKMesh::New();
    cv::Vec3d first;
    cv::Vec3d middle;
    cv::Vec3d last;
    for (const auto& z : ProgressWrap(range(volume->numSlices()), "Slice:")) {
        // Get the slice and blur it
        auto slice =
            vc::QuantizeImage(volume->getSliceDataCopy(z), CV_8UC1, false);
        cv::GaussianBlur(slice, slice, {gaussianKernel, gaussianKernel}, 0);
        if (bilateral) {
            cv::bilateralFilter(slice.clone(), slice, gaussianKernel, 75, 75);
        }

        // Run Canny Edge Detect
        cv::Mat cannySlice;
        cv::Canny(slice, cannySlice, minVal, maxVal, aperture, true);

        cv::Mat processed;
        cannySlice.copyTo(processed, mask);

        // Replace canny edges with contour edges
        if (contour) {
            // Apply closing to fill holes and gaps.
            cv::Mat kernel = cv::Mat::ones(closingKernel, closingKernel, CV_8U);
            cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point2i>> contours;
            cv::findContours(
                processed, contours, cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE);

            processed = cv::Mat::zeros(slice.size(), CV_8UC1);
            cv::drawContours(processed, contours, -1, {255});
        }

        // Keep all edges
        if (edge == "N") {
            for (const auto pt : range2D(processed.rows, processed.cols)) {
                const auto& x = pt.second;
                const auto& y = pt.first;
                if (processed.at<uint8_t>(y, x) > 0) {
                    middle = {
                        static_cast<double>(x), static_cast<double>(y),
                        static_cast<double>(z)};
                    mesh->SetPoint(mesh->GetNumberOfPoints(), middle.val);
                }
            }
            continue;
        }

        // To allow for choosing between left-right and top-down, use
        // generalized inner and outer indices instead of x and y values.
        // This simply changes the order in which the points are iterated over.
        // There is no transformation applied to the image or the resulting
        // points.
        int x, y, outerCount, outerIdx, innerCount, innerIdx;

        // If looking for edges left to right, the outer loop iterates over y
        // values and the inner loop iterates over x values.
        // If going top to bottom, it is the opposite.
        if (edge == "T" || edge == "B") {
            outerCount = processed.cols;
            innerCount = processed.rows;
        } else {
            outerCount = processed.rows;
            innerCount = processed.cols;
        }

        for (outerIdx = 0; outerIdx < outerCount; outerIdx++) {
            // Get the first
            auto haveFirst = false;
            for (innerIdx = 0; innerIdx < innerCount; innerIdx++) {

                // Determine actual (x, y) position based on loop positions.
                // In either case (left-right or top-down), can flip the
                // direction to become right-left or bottom-up by reversing the
                // inner loop position.
                if (edge == "T" || edge == "B") {
                    x = outerIdx;
                    y = (edge == "R" || edge == "B") ? innerCount - innerIdx - 1
                                                     : innerIdx;
                } else {
                    x = (edge == "R" || edge == "B") ? innerCount - innerIdx - 1
                                                     : innerIdx;
                    y = outerIdx;
                }

                if (!haveFirst && processed.at<uint8_t>(y, x) != 0) {
                    first = {
                        static_cast<double>(x), static_cast<double>(y),
                        static_cast<double>(z)};
                    last = first;
                    haveFirst = true;
                    continue;
                }

                if (midpoint && haveFirst && processed.at<uint8_t>(y, x) != 0) {
                    last = {
                        static_cast<double>(x), static_cast<double>(y),
                        static_cast<double>(z)};
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

void ShowCanny()
{
    cv::namedWindow(outputWindowName, cv::WINDOW_NORMAL);
    cv::namedWindow(settingsWindowName);
    cv::startWindowThread();
    cv::createTrackbar(
        "Blur Size:", settingsWindowName, &blurSlider, 30, CannyThreshold);
    cv::createTrackbar(
        "Min Threshold:", settingsWindowName, &minVal, 255, CannyThreshold);
    cv::createTrackbar(
        "Max Threshold:", settingsWindowName, &maxVal, 255, CannyThreshold);
    cv::createTrackbar(
        "Aperture Size:", settingsWindowName, &apertureSlider, 2,
        CannyThreshold);
    cv::createTrackbar(
        "Slice: ", settingsWindowName, &sliceIdx, volume->numSlices() - 1,
        CannyThreshold);
    if (contour) {
        cv::createTrackbar(
            "Closing Size: ", settingsWindowName, &closingSlider, 32,
            CannyThreshold);
    }

    CannyThreshold(0, nullptr);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void CannyThreshold(int, void*)
{
    auto slice = volume->getSliceDataCopy(sliceIdx);
    src = vc::QuantizeImage(slice, CV_8U, false);

    gaussianKernel = 2 * blurSlider + 1;
    aperture = 2 * apertureSlider + 3;
    cv::Mat canny;
    cv::GaussianBlur(src, canny, {gaussianKernel, gaussianKernel}, 0);
    if (bilateral) {
        src = canny;
        canny = cv::Mat();
        cv::bilateralFilter(src, canny, gaussianKernel, 75, 75);
    }

    // Run Canny Edge Detect
    cv::Canny(canny, canny, minVal, maxVal, aperture, true);

    // Apply the mask to the canny image
    dst = cv::Scalar::all(0);
    canny.copyTo(dst, mask);

    // Draw contours
    if (contour) {
        closingKernel = 2 * closingSlider + 1;
        // Apply closing to fill holes and gaps.
        cv::Mat kernel = cv::Mat::ones(closingKernel, closingKernel, CV_8U);
        cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(
            dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        dst = cv::Mat::zeros(src.size(), CV_8UC3);
        cv::drawContours(dst, contours, -1, {0, 255, 0});
    }

    src = vc::QuantizeImage(slice, CV_8U);
    src = vc::ColorConvertImage(src, dst.channels());
    cv::addWeighted(src, 0.5, dst, 0.5, 0, dst);

    cv::imshow(outputWindowName, dst);
}
