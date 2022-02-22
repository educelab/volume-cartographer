#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
// projection.cpp
// Seth Parker 10/2015
// Project the mesh onto a volume in order to check the quality of segmentation
#include <iomanip>
#include <iostream>
#include <map>

#include <QApplication>
#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkCutter.h>
#include <vtkPlane.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>

#include "ProjectionViewerWindow.hpp"
#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/ITK2VTK.hpp"

namespace po = boost::program_options;
namespace fs = volcart::filesystem;
namespace vc = volcart;

auto main(int argc, char* argv[]) -> int
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
        ("visualize", "Show projection interactive visualization before running on entire volume")
        ("color,c", po::value<int>()->default_value(0),
         "Color of the intersection line:\n"
             "  0 = White\n"
             "  1 = Red\n"
             "  2 = Green\n"
             "  3 = Blue\n")
        ("thickness,t", po::value<int>()->default_value(1), "Line thickness")
        ("intersect-only", "Draws the intersection on a black image")
        ("visualize-ppm-intersection", po::value<std::string>(), "PPM to show in addition to the slices view")
        ("ppm-image-overlay", po::value<std::string>(), "Image to overlay in the PPM space when viewing how the slice intersects the PPM. Default: <ppm>_mask.png");

    po::options_description all("Usage");
    all.add(required).add(visOptions);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if ((parsed.count("help") > 0) || argc < 4) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Get options
    vc::ProjectionSettings projectionSettings;
    auto meshPaths = parsed["input-mesh"].as<std::vector<std::string>>();
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    fs::path outputDir = parsed["output-dir"].as<std::string>();
    projectionSettings.intersectOnly = parsed.count("intersect-only") > 0;
    projectionSettings.thickness = parsed["thickness"].as<int>();
    if (parsed.count("visualize-ppm-intersection") > 0) {
        projectionSettings.visualizePPMIntersection =
            parsed["visualize-ppm-intersection"].as<std::string>();
        if (parsed.count("ppm-image-overlay") > 0) {
            projectionSettings.ppmImageOverlay =
                parsed["ppm-image-overlay"].as<std::string>();
        } else {
            auto ppmPath = vc::filesystem::path(
                projectionSettings.visualizePPMIntersection);
            auto maskPath =
                ppmPath.parent_path() / (ppmPath.stem().string() + "_mask.png");
            projectionSettings.ppmImageOverlay = maskPath.string();
        }
    }

    // Color Option
    auto colorOpt = static_cast<Color>(parsed["color"].as<int>());
    switch (colorOpt) {
        case Color::White:
            projectionSettings.color = WHITE;
            break;
        case Color::Red:
            projectionSettings.color = RED;
            break;
        case Color::Green:
            projectionSettings.color = GREEN;
            break;
        case Color::Blue:
            projectionSettings.color = BLUE;
            break;
    }

    // Load the volume package
    vc::VolumePkg volpkg(volpkgPath);

    // Load the volume
    vc::Volume::Pointer volume;
    try {
        if (parsed.count("volume") > 0) {
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
    for (const auto& meshPath : meshPaths) {
        reader.setPath(meshPath);
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
        for (auto& mesh : meshes) {
            append->AddInputData(mesh);
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
    cutPlane->SetOrigin(width / 2.0, height / 2.0, 0);
    cutPlane->SetNormal(0, 0, 1);
    projectionSettings.zMin =
        static_cast<int>(std::floor(vtkMesh->GetBounds()[4]));
    projectionSettings.zMax =
        static_cast<int>(std::ceil(vtkMesh->GetBounds()[5]));

    // Bounds checks
    if (projectionSettings.zMin < 0) {
        projectionSettings.zMin = 0;
    }
    if (projectionSettings.zMax >= volume->numSlices()) {
        projectionSettings.zMax = volume->numSlices() - 1;
    }
    if (projectionSettings.zMin == projectionSettings.zMax) {
        projectionSettings.zMax += 1;
    }

    // Setup cutting and stripping pipeline
    auto cutter = vtkSmartPointer<vtkCutter>::New();
    cutter->SetCutFunction(cutPlane);
    cutter->SetInputData(vtkMesh);

    auto stripper = vtkSmartPointer<vtkStripper>::New();
    stripper->SetInputConnection(cutter->GetOutputPort());

    if (parsed.count("visualize") > 0) {
        QApplication app(argc, argv);
        QGuiApplication::setApplicationDisplayName(
            ProjectionViewerWindow::tr("Projection Viewer"));
        ProjectionViewerWindow viewer(
            &projectionSettings, cutPlane, stripper, volume);
        viewer.show();
        QApplication::exec();
    }

    // Iterate over every z-index in the range between zMin and zMax
    // Cut the mesh and draw its corresponding intersection onto a new output
    // image
    cv::Mat outputImg;
    std::vector<cv::Point> contour;
    for (const auto& zIdx : vc::ProgressWrap(
             vc::range(projectionSettings.zMin, projectionSettings.zMax),
             "vc::projection::Projecting:")) {
        // Cut the mesh and get the intersection
        cutPlane->SetOrigin(width / 2.0, height / 2.0, zIdx);
        stripper->Update();
        auto* intersection = stripper->GetOutput();

        // Setup the output image
        if (projectionSettings.intersectOnly) {
            outputImg = cv::Mat::zeros(height, width, CV_8UC3);
        } else {
            outputImg = volume->getSliceDataCopy(zIdx);
            outputImg.convertTo(outputImg, CV_8U, MAX_8BPC / MAX_16BPC);
            cv::cvtColor(outputImg, outputImg, cv::COLOR_GRAY2BGR);
        }

        // Draw the intersections
        for (auto cId = 0; cId < intersection->GetNumberOfCells(); ++cId) {
            auto* inputCell = intersection->GetCell(cId);

            contour.clear();
            for (auto pIt = 0; pIt < inputCell->GetNumberOfPoints(); ++pIt) {
                auto pId = inputCell->GetPointId(pIt);
                contour.emplace_back(cv::Point(
                    static_cast<int>(intersection->GetPoint(pId)[0]),
                    static_cast<int>(intersection->GetPoint(pId)[1])));
            }

            cv::polylines(
                outputImg, contour, false, projectionSettings.color,
                projectionSettings.thickness, cv::LINE_AA);
        }

        // Save the output to the provided directory
        std::stringstream filename;
        filename << std::setw(padding) << std::setfill('0') << zIdx << ".png";
        auto path = outputDir / filename.str();
        cv::imwrite(path.string(), outputImg);
    }

    return EXIT_SUCCESS;
}

#pragma clang diagnostic pop
