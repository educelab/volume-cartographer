// layers.cpp

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <vtkCleanPolyData.h>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/external/GetMemorySize.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/SmoothNormals.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/LayerTexture.hpp"
#include "vc/texturing/PPMGenerator.hpp"

using namespace volcart;
namespace fs = boost::filesystem;
namespace po = boost::program_options;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 4;
// Number of vertices per square millimeter
static constexpr double SAMPLING_DENSITY_FACTOR = 50;
// Square Micron to square millimeter conversion factor
static constexpr double UM_TO_MM = 0.001 * 0.001;
// Min. number of points required to do flattening
static constexpr uint16_t CLEANER_MIN_REQ_POINTS = 100;

int main(int argc, char* argv[])
{
    std::cout << "vc_layers" << std::endl;
    ///// Parse the command line options /////
    fs::path volpkgPath, outputPath, ppmPath;
    std::string segID;
    double radius, interval;
    Direction aDirectionOption;

    try {
        // All command line options
        // clang-format off
        po::options_description options("Options");
        options.add_options()
            ("help,h", "Show this message")
            ("volpkg,v", po::value<std::string>()->required(),
                "Path to the volume package")
            ("seg,s", po::value<std::string>()->required(),
                "Segmenation ID number")
            ("radius,r", po::value<int>()->required(), "Texture search radius")
            ("interval,i", po::value<double>()->default_value(1.0))
            ("direction,d", po::value<int>()->default_value(0),
                "Sample Direction:\n"
                "  0 = Omni\n"
                "  1 = Positive\n"
                "  2 = Negative\n")
            ("output-dir,o", po::value<std::string>(),
                "Output directory for layer images.")
            ("output-ppm", po::value<std::string>(),
                "Output path for generated per-pixel map.");
        // clang-format on

        // parsedOptions will hold the values of all parsed options as a Map
        po::variables_map parsedOptions;
        po::store(
            po::command_line_parser(argc, argv).options(options).run(),
            parsedOptions);

        // Show the help message
        if (parsedOptions.count("help") || argc < 2) {
            std::cout << options << std::endl;
            return EXIT_SUCCESS;
        }

        // Warn of missing options
        try {
            po::notify(parsedOptions);
        } catch (po::error& e) {
            std::cerr << "ERROR: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }

        // Get the parsed options
        volpkgPath = parsedOptions["volpkg"].as<std::string>();
        segID = parsedOptions["seg"].as<std::string>();
        radius = parsedOptions["radius"].as<int>();
        interval = parsedOptions["interval"].as<double>();
        aDirectionOption =
            static_cast<Direction>(parsedOptions["direction"].as<int>());

        // Check for output file
        if (parsedOptions.count("output-dir")) {
            outputPath =
                fs::canonical(parsedOptions["output-dir"].as<std::string>());
            if (!fs::is_directory(outputPath) || !fs::exists(outputPath)) {
                throw IOException(
                    "Provided output path is not a directory or does not "
                    "exist");
            }
        }

        // Check for output ppm path
        if (parsedOptions.count("output-ppm")) {
            ppmPath = parsedOptions["output-ppm"].as<std::string>();
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the volume package /////
    if (fs::exists(volpkgPath) ||
        (fs::canonical(volpkgPath).extension() != ".volpkg")) {
        volpkgPath = fs::canonical(volpkgPath);
    } else {
        std::cerr << "ERROR: Volume package does not exist/not recognized at "
                     "provided path: "
                  << volpkgPath << std::endl;
        return EXIT_FAILURE;
    }

    VolumePkg vpkg(volpkgPath);
    if (vpkg.getVersion() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion()
                  << " but this program requires a version "
                  << std::to_string(VOLPKG_SUPPORTED_VERSION) << "."
                  << std::endl;
        return EXIT_FAILURE;
    }
    double cacheBytes = 0.45 * SystemMemorySize();
    vpkg.volume()->setCacheMemoryInBytes(static_cast<size_t>(cacheBytes));

    ///// Set the segmentation ID /////
    vpkg.setActiveSegmentation(segID);
    fs::path meshName = vpkg.getMeshPath();

    // declare pointer to new Mesh object
    volcart::io::PLYReader reader(meshName);
    try {
        reader.read();
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    auto input = reader.getMesh();

    // Calculate sampling density
    auto voxelToMicron = std::pow(vpkg.volume()->voxelSize(), 2);
    auto area = meshmath::SurfaceArea(input) * voxelToMicron * UM_TO_MM;
    auto vertCount = static_cast<uint16_t>(SAMPLING_DENSITY_FACTOR * area);
    vertCount = (vertCount < CLEANER_MIN_REQ_POINTS) ? CLEANER_MIN_REQ_POINTS
                                                     : vertCount;

    // Convert to polydata
    auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
    volcart::meshing::ITK2VTK(input, vtkMesh);

    // Decimate using ACVD
    std::cout << "Resampling mesh..." << std::endl;
    auto acvdMesh = vtkSmartPointer<vtkPolyData>::New();
    volcart::meshing::ACVD(vtkMesh, acvdMesh, vertCount);

    // Merge Duplicates
    // Note: This merging has to be the last in the process chain for some
    // really weird reason. - SP
    auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    cleaner->SetInputData(acvdMesh);
    cleaner->Update();

    auto itkACVD = volcart::ITKMesh::New();
    volcart::meshing::VTK2ITK(cleaner->GetOutput(), itkACVD);

    // ABF flattening
    std::cout << "Computing parameterization..." << std::endl;
    volcart::texturing::AngleBasedFlattening abf(itkACVD);
    // abf.setABFMaxIterations(5);
    abf.compute();

    // Get uv map
    volcart::UVMap uvMap = abf.getUVMap();
    auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<size_t>(std::ceil(width / uvMap.ratio().aspect));

    texturing::PPMGenerator p;
    p.setDimensions(height, width);
    p.setMesh(itkACVD);
    p.setUVMap(uvMap);
    p.compute();

    texturing::LayerTexture s;
    s.setVolume(vpkg.volume());
    s.setPerPixelMap(p.getPPM());
    s.setSamplingRadius(radius);
    s.setSamplingInterval(interval);
    s.setSamplingDirection(aDirectionOption);

    // Setup rendering
    volcart::Rendering rendering;
    rendering.setTexture(s.compute());
    rendering.setMesh(itkACVD);

    std::cout << "Writing layers..." << std::endl;
    fs::path filepath;
    for (size_t i = 0; i < rendering.getTexture().numberOfImages(); ++i) {
        filepath = outputPath / (std::to_string(i) + ".png");
        cv::imwrite(filepath.string(), rendering.getTexture().image(i));
    }

    // Write ppm
    if (!ppmPath.empty()) {
        std::cout << "Writing PPM..." << std::endl;
        volcart::PerPixelMap::WritePPM(ppmPath, p.getPPM());
    }

    return EXIT_SUCCESS;
}  // end main
