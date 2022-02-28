#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-identifier-length"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
#include <iostream>
#include <vector>

#include <QApplication>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "CannyViewerWindow.hpp"
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

using vc::range;
using vc::range2D;

auto main(int argc, char* argv[]) -> int
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

    // Canny values
    vc::CannySettings cannySettings;

    // Get options
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    fs::path outputPath = parsed["output-file"].as<std::string>();
    cannySettings.minThreshold = parsed["threshold-min"].as<int>();
    cannySettings.maxThreshold = parsed["threshold-max"].as<int>();
    cannySettings.contour = parsed.count("use-contour") > 0;
    cannySettings.bilateral = parsed.count("bilateral") > 0;

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

    cv::Mat mask;
    if (parsed.count("mask") > 0) {
        cannySettings.mask =
            cv::imread(parsed["mask"].as<std::string>(), CV_8UC1);
    }

    if (parsed.count("visualize") > 0) {
        QApplication app(argc, argv);
        QGuiApplication::setApplicationDisplayName(
            CannyViewerWindow::tr("Canny Viewer"));
        CannyViewerWindow viewer(&cannySettings, volume);
        viewer.show();
        QApplication::exec();
    }

    auto midpoint = parsed.count("calculate-midpoint") > 0;

    // Segment
    std::cout << "Segmenting surface..." << std::endl;
    auto mesh = vc::ITKMesh::New();
    cv::Vec3d first;
    cv::Vec3d middle;
    cv::Vec3d last;
    for (const auto& z : ProgressWrap(range(volume->numSlices()), "Slice:")) {
        // Get the slice
        auto slice =
            vc::QuantizeImage(volume->getSliceDataCopy(z), CV_8UC1, false);

        cv::Mat processed = vc::Canny(slice, cannySettings);

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

#pragma clang diagnostic pop
