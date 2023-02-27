#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-identifier-length"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
#include <iostream>
#include <vector>

#include <QApplication>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vtkAppendPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkCutter.h>
#include <vtkLine.h>
#include <vtkPointData.h>
#include <vtkProbeFilter.h>
#include <vtkSmartPointer.h>

#include "CannyViewerWindow.hpp"
#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/ImageConversion.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/String.hpp"
#include "vc/meshing/ITK2VTK.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;

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
        ("visualize", "Show Canny visualization before segmenting")
        ("blur-size", po::value<int>()->default_value(1), "Blur size")
        ("threshold-min", po::value<int>()->default_value(100), "Minimum intensity gradient")
        ("threshold-max", po::value<int>()->default_value(150), "Maximum intensity gradient")
        ("aperture-size", po::value<int>()->default_value(0), "Aperture size")
        ("use-contour,c", "If enabled, draw contour around canny edges before projection")
        ("closing-size", po::value<int>()->default_value(4), "Closing size for contour")
        ("bilateral,b", "If enabled, bilateral filter image")
        ("project-from,f", po::value<std::string>()->default_value("L"), "Direction to segment from: (L)eft, (R)ight, (T)op, (B)ottom, (M)esh normals, (I)nverted mesh normals, (N)one")
        ("calculate-midpoint", "When enabled, try to find a surface midpoint")
        ("from-mesh,m", po::value<std::vector<std::string>>(), "Use a mesh to seed the segmentation. May be specified multiple times.")
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

    // Canny values
    vc::CannySettings cannySettings;

    // Get options
    const fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    const fs::path outputPath = parsed["output-file"].as<std::string>();

    cannySettings.blurSize = parsed["blur-size"].as<int>();
    cannySettings.minThreshold = parsed["threshold-min"].as<int>();
    cannySettings.maxThreshold = parsed["threshold-max"].as<int>();
    cannySettings.apertureSize = parsed["aperture-size"].as<int>();
    cannySettings.contour = parsed.count("use-contour") > 0;
    cannySettings.closingSize = parsed["closing-size"].as<int>();
    cannySettings.bilateral = parsed.count("bilateral") > 0;
    cannySettings.midpoint = parsed.count("calculate-midpoint") > 0;

    auto fromStr = vc::to_upper_copy(parsed["project-from"].as<std::string>());
    auto from = fromStr[0];
    if (from != 'L' && from != 'R' && from != 'T' && from != 'B' &&
        from != 'M' && from != 'I' && from != 'N') {
        std::cerr << "ERROR: projection-from must be one of L,R,T,B,M,I,N\n";
        return EXIT_FAILURE;
    }
    if ((from == 'M' || from == 'I') && parsed.count("from-mesh") == 0) {
        std::cerr << "ERROR: projection-from=[M,I] requires --from-mesh to be "
                     "specified\n";
        return EXIT_FAILURE;
    }
    cannySettings.projectionFrom = from;

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

    cannySettings.zMin = 0;
    cannySettings.zMax = volume->numSlices() - 1;

    if (parsed.count("mask") > 0) {
        cannySettings.mask =
            cv::imread(parsed["mask"].as<std::string>(), CV_8UC1);
    }

    if (parsed.count("from-mesh") > 0) {
        cannySettings.fromMeshes = parsed["from-mesh"].as<std::vector<std::string>>();
    }

    /**************************************************************************/
    /******************************** MESHES **********************************/

    // Get meshes
    std::cout << "Loading meshes..." << std::endl;
    std::vector<vtkSmartPointer<vtkPolyData>> meshes;
    for (const auto& meshPath : cannySettings.fromMeshes) {
        auto meshFile = vc::ReadMesh(meshPath);
        auto vtkMesh = vcm::ITK2VTK(meshFile.mesh);
        meshes.push_back(vtkMesh);
    }

    // Combine meshes if we have multiple
    vtkSmartPointer<vtkPolyData> vtkMesh;
    if (meshes.size() > 1) {
        // Append all the meshes into a single polydata
        auto append = vtkSmartPointer<vtkAppendPolyData>::New();
        for (const auto& mesh : meshes) {
            append->AddInputData(mesh);
        }
        append->Update();

        // Clean it up
        auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
        cleaner->SetInputConnection(append->GetOutputPort());
        cleaner->Update();

        vtkMesh = cleaner->GetOutput();
    } else if (meshes.size() == 1) {
        vtkMesh = meshes[0];
    }

    vtkSmartPointer<vtkCutter> cutter;
    vtkSmartPointer<vtkPlane> plane;
    if (vtkMesh != nullptr) {
        cutter = vtkSmartPointer<vtkCutter>::New();
        plane = vtkSmartPointer<vtkPlane>::New();

        auto zMin = static_cast<int>(std::floor(vtkMesh->GetBounds()[4]));
        auto zMax = static_cast<int>(std::ceil(vtkMesh->GetBounds()[5]));

        // Bounds checks
        if (zMin < 0) {
            zMin = 0;
        }
        if (zMax >= volume->numSlices()) {
            zMax = volume->numSlices() - 1;
        }
        if (zMin == zMax) {
            zMax += 1;
        }

        cannySettings.zMin = zMin;
        cannySettings.zMax = zMax;

        plane->SetOrigin(
            static_cast<double>(volume->sliceWidth()) / 2,
            static_cast<double>(volume->sliceHeight()) / 2,
            0
        );
        plane->SetNormal(0, 0, 1);

        cutter->SetInputData(vtkMesh);
        cutter->SetCutFunction(plane);
        cutter->Update();
    }
    /**************************************************************************/

    if (parsed.count("visualize") > 0) {
        const QApplication app(argc, argv);
        QGuiApplication::setApplicationDisplayName(QObject::tr("Canny Viewer"));
        CannyViewerWindow viewer(&cannySettings, volume);
        viewer.show();
        if (QApplication::exec() == EXIT_FAILURE) {
            return EXIT_FAILURE;
        }
    }

    // check that if project from is 'M' or 'I' that we have a mesh
    if ((cannySettings.projectionFrom == 'M' ||
         cannySettings.projectionFrom == 'I') &&
        vtkMesh == nullptr) {
        std::cerr << "ERROR: projection-from=[M,I] requires --from-mesh to be "
                     "specified\n";
        return EXIT_FAILURE;
    }

    // Segment
    std::cout << "Segmenting surface..." << std::endl;
    auto mesh = vc::ITKMesh::New();
    cv::Vec3d first;
    cv::Vec3d middle;
    cv::Vec3d last;
    auto zMin = static_cast<int>(cannySettings.zMin);
    auto zMax = static_cast<int>(cannySettings.zMax);
    for (const auto& z : ProgressWrap(range(zMin, zMax), "Slice:")) {
        // Get the slice
        auto slice =
            vc::QuantizeImage(volume->getSliceDataCopy(z), CV_8UC1, false);

        auto processed = vc::Canny(slice, cannySettings);

        // Keep all edges
        if (cannySettings.projectionFrom == 'N') {
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

        // Build the set of rays that will be projected to find the edges
        std::vector<cv::Vec2d> rayBases;
        std::vector<cv::Vec2d> rayOrigins;

        if (cannySettings.projectionFrom == 'L') {
            for (auto y : range(processed.rows)) {
                rayOrigins.push_back({0, static_cast<double>(y)});
                rayBases.push_back({1, 0});
            }
        } else if (cannySettings.projectionFrom == 'R') {
            for (auto y : range(processed.rows)) {
                rayOrigins.push_back({static_cast<double>(processed.cols - 1), static_cast<double>(y)});
                rayBases.push_back({-1, 0});
            }
        } else if (cannySettings.projectionFrom == 'T') {
            for (auto x : range(processed.cols)) {
                rayOrigins.push_back({static_cast<double>(x), 0});
                rayBases.push_back({0, 1});
            }
        } else if (cannySettings.projectionFrom == 'B') {
            for (auto x : range(processed.cols)) {
                rayOrigins.push_back({static_cast<double>(x), static_cast<double>(processed.rows - 1)});
                rayBases.push_back({0, -1});
            }
        } else if (cannySettings.projectionFrom == 'M' || cannySettings.projectionFrom == 'I') {
            if (plane == nullptr || cutter == nullptr) {
                std::cerr << "Error: Projection from mesh requested but mesh "
                             "cutter not initialized\n";
                return EXIT_FAILURE;
            }
            // update plane Z
            plane->SetOrigin(0, 0, static_cast<double>(z));
            cutter->Update();

            // get cutter output
            const vtkSmartPointer<vtkPolyData> cutterOutput =
                cutter->GetOutput();

            // vtk list of points
            auto points = vtkSmartPointer<vtkPoints>::New();

            // go through line segments
            for (auto i = 0; i < cutterOutput->GetNumberOfCells(); ++i) {
                // get cell
                const vtkSmartPointer<vtkCell> cell = cutterOutput->GetCell(i);

                // get points as cv::Vec3d
                const cv::Vec3d p0{cell->GetPoints()->GetPoint(0)};
                const cv::Vec3d p1{cell->GetPoints()->GetPoint(1)};

                const auto length = cv::norm(p1 - p0);
                const cv::Vec3d step = (p1 - p0) / length;

                for (auto t = 0; t < static_cast<int>(length); ++t) {
                    auto p = p0 + step * t;
                    points->InsertNextPoint(p.val);
                }
            }

            // if zero points, skip
            if (points->GetNumberOfPoints() == 0) {
                continue;
            }

            // create polydata
            const auto pointsPolyData = vtkSmartPointer<vtkPolyData>::New();
            pointsPolyData->SetPoints(points);

            // use probe filter
            const auto probeFilter = vtkSmartPointer<vtkProbeFilter>::New();
            probeFilter->SetInputData(pointsPolyData);
            probeFilter->SetSourceData(vtkMesh);
            probeFilter->Update();

            // get probe output
            const vtkSmartPointer<vtkDataSet> probeOutput =
                probeFilter->GetOutput();

            // get normal vectors
            const vtkSmartPointer<vtkDataArray> normals =
                probeOutput->GetPointData()->GetNormals();

            // check if normals exist
            if (normals == nullptr) {
                std::cerr << "Error: Input mesh has no normals\n";
                return EXIT_FAILURE;
            }

            // go through points and normals
            for (auto i = 0; i < probeOutput->GetNumberOfPoints(); ++i) {
                // get point
                cv::Vec3d p{probeOutput->GetPoint(i)};
                // get normal
                cv::Vec3d n{normals->GetTuple(i)};
                // check if normal is all zeros
                if (n[0] == 0 && n[1] == 0 && n[2] == 0) {
                    continue;
                }
                // project normal to xy plane
                n[2] = 0;
                // normalize
                n = n / cv::norm(n);
                // invert normal if needed
                if (cannySettings.projectionFrom == 'I') {
                    n = -n;
                }
                // add point to ray origins after converting to 2d
                rayOrigins.push_back({p[0], p[1]});
                // add normal to ray bases after converting to 2d
                rayBases.push_back({n[0], n[1]});
            }
        }

        for (auto r : range(rayBases.size())) {
            const auto& rayBasis = rayBases[r];
            const auto& rayOrigin = rayOrigins[r];

            // Get the first
            auto haveFirst = false;

            auto pt = rayOrigin;
            int xI = static_cast<int>(pt[0]);
            int yI = static_cast<int>(pt[1]);
            while (
                xI >= 0
                && xI < processed.cols
                && yI >= 0
                && yI < processed.rows
            ) {
                // if point is on detected edge
                if (!haveFirst && processed.at<uint8_t>(yI, xI) != 0) {
                    // set first
                    first = {pt[0], pt[1], static_cast<double>(z)};
                    last = first;
                    haveFirst = true;
                    continue;
                }

                if (cannySettings.midpoint && haveFirst && processed.at<uint8_t>(yI, xI) != 0) {
                    last = {pt[0], pt[1], static_cast<double>(z)};
                }

                if (!cannySettings.midpoint && haveFirst) {
                    break;
                }

                // move point along ray
                pt += rayBasis * 0.5;
                xI = static_cast<int>(pt[0]);
                yI = static_cast<int>(pt[1]);
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
