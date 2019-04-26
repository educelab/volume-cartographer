// projection.cpp
// Seth Parker 10/2015
// Project the mesh onto a volume in order to check the quality of segmentation
#include <iostream>
#include <limits>
#include <map>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vtkAppendPolyData.h>
#include <vtkCell.h>
#include <vtkCleanPolyData.h>
#include <vtkCutter.h>
#include <vtkPlane.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>

#include "vc/core/io/OBJReader.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/ITK2VTK.hpp"

static const double MAX_8BPC = std::numeric_limits<uint8_t>::max();
static const double MAX_16BPC = std::numeric_limits<uint16_t>::max();

static const cv::Scalar WHITE{255, 255, 255};
static const cv::Scalar BLUE{255, 0, 0};
static const cv::Scalar GREEN{0, 255, 0};
static const cv::Scalar RED{0, 0, 255};

enum class Color { White = 0, Red, Green, Blue };

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vc = volcart;

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("input-mesh,i", po::value<std::vector<std::string>>()->required(),
             "OBJ mesh to project onto the volume. Can be specified "
             "multiple times.")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("volume", po::value<std::string>(),
             "Volume to use for texturing. Default: First volume")
        ("output-dir,o", po::value<std::string>()->required(),
             "Output directory");

    po::options_description visOptions("Visualization Options");
    visOptions.add_options()
        ("color,c", po::value<int>()->default_value(0),
         "Color of the intersection line:\n"
             "  0 = White\n"
             "  1 = Red\n"
             "  2 = Green\n"
             "  3 = Blue\n")
        ("thickness,t", po::value<int>()->default_value(1), "Line thickness")
        ("intersect-only", "Draws the intersection on a black image");

    po::options_description all("Usage");
    all.add(required).add(visOptions);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 4) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::logger->error(e.what());
        return EXIT_FAILURE;
    }

    // Get options
    auto meshPaths = parsed["input-mesh"].as<std::vector<std::string>>();
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    fs::path outputDir = parsed["output-dir"].as<std::string>();
    auto intersectOnly = parsed.count("intersect-only") > 0;
    auto thickness = parsed["thickness"].as<int>();

    // Color Option
    auto colorOpt = static_cast<Color>(parsed["color"].as<int>());
    cv::Scalar color;
    switch (colorOpt) {
        case Color::White:
            color = WHITE;
            break;
        case Color::Red:
            color = RED;
            break;
        case Color::Green:
            color = GREEN;
            break;
        case Color::Blue:
            color = BLUE;
            break;
    }

    // Load the volume package
    vc::VolumePkg volpkg(volpkgPath);

    // Load the volume
    vc::Volume::Pointer volume;
    try {
        if (parsed.count("volume")) {
            volume = volpkg.volume(parsed["volume"].as<std::string>());
        } else {
            volume = volpkg.volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct."
                  << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    auto width = volume->sliceWidth();
    auto height = volume->sliceHeight();
    auto padding = static_cast<int>(std::to_string(volume->numSlices()).size());

    // Get meshes
    std::cout << "Loading meshes..." << std::endl;
    vc::io::OBJReader reader;
    std::vector<vtkSmartPointer<vtkPolyData>> meshes;
    for (const auto& p : meshPaths) {
        reader.setPath(p);
        auto mesh = reader.read();
        auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        vc::meshing::ITK2VTK(mesh, vtkMesh);
        meshes.push_back(vtkMesh);
    }

    // Combine meshes if we have multiple
    vtkSmartPointer<vtkPolyData> vtkMesh;
    if (meshes.size() > 1) {
        // Append all of the meshes into a single polydata
        auto append = vtkSmartPointer<vtkAppendPolyData>::New();
        for (auto& m : meshes) {
            append->AddInputData(m);
        }
        append->Update();

        // Clean it up
        auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
        cleaner->SetInputConnection(append->GetOutputPort());
        cleaner->Update();

        vtkMesh = cleaner->GetOutput();
    } else {
        vtkMesh = meshes[0];
    }

    // Setup intersection plane
    vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
    cutPlane->SetOrigin(width / 2, height / 2, 0);
    cutPlane->SetNormal(0, 0, 1);
    auto z_min = static_cast<int>(std::floor(vtkMesh->GetBounds()[4]));
    auto z_max = static_cast<int>(std::ceil(vtkMesh->GetBounds()[5]));

    // Bounds checks
    if (z_min < 0) {
        z_min = 0;
    }
    if (z_max >= volume->numSlices()) {
        z_max = volume->numSlices() - 1;
    }
    if (z_min == z_max) {
        z_max += 1;
    }

    // Setup cutting and stripping pipeline
    auto cutter = vtkSmartPointer<vtkCutter>::New();
    cutter->SetCutFunction(cutPlane);
    cutter->SetInputData(vtkMesh);

    auto stripper = vtkSmartPointer<vtkStripper>::New();
    stripper->SetInputConnection(cutter->GetOutputPort());

    // Iterate over every z-index in the range between z_min and z_max
    // Cut the mesh and draw its corresponding intersection onto a new output
    // image
    cv::Mat outputImg;
    std::vector<cv::Point> contour;
    for (int it = z_min; it < z_max; ++it) {
        std::cerr << "vc::projection::Projecting " << it << "\r" << std::flush;

        // Cut the mesh and get the intersection
        cutPlane->SetOrigin(width / 2, height / 2, it);
        stripper->Update();
        auto intersection = stripper->GetOutput();

        // Setup the output image
        if (intersectOnly) {
            outputImg = cv::Mat::zeros(height, width, CV_8UC3);
        } else {
            outputImg = volume->getSliceDataCopy(it);
            outputImg.convertTo(outputImg, CV_8U, MAX_8BPC / MAX_16BPC);
            cv::cvtColor(outputImg, outputImg, cv::COLOR_GRAY2BGR);
        }

        // Draw the intersections
        for (auto c_id = 0; c_id < intersection->GetNumberOfCells(); ++c_id) {
            auto inputCell = intersection->GetCell(c_id);

            contour.clear();
            for (auto p_it = 0; p_it < inputCell->GetNumberOfPoints(); ++p_it) {
                auto p_id = inputCell->GetPointId(p_it);
                contour.emplace_back(cv::Point(
                    static_cast<int>(intersection->GetPoint(p_id)[0]),
                    static_cast<int>(intersection->GetPoint(p_id)[1])));
            }

            cv::polylines(
                outputImg, contour, false, color, thickness, cv::LINE_AA);
        }

        // Save the output to the provided directory
        std::stringstream filename;
        filename << std::setw(padding) << std::setfill('0') << it << ".png";
        auto path = outputDir / filename.str();
        cv::imwrite(path.string(), outputImg);
    }
    std::cerr << std::endl;

    return EXIT_SUCCESS;
}
