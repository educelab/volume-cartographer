// layers.cpp

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <vtkCleanPolyData.h>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/external/GetMemorySize.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/meshing/SmoothNormals.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/LayerTexture.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 5;
// Number of vertices per square millimeter
static constexpr double SAMPLING_DENSITY_FACTOR = 50;
// Square Micron to square millimeter conversion factor
static constexpr double UM_TO_MM = 0.001 * 0.001;
// Min. number of points required to do flattening
static constexpr uint16_t CLEANER_MIN_REQ_POINTS = 100;

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("seg,s", po::value<std::string>()->required(), "Segmentation ID")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: First volume.")
        ("output-dir,o", po::value<std::string>()->required(),
            "Output directory for layer images.")
        ("output-ppm", po::value<std::string>(),
            "Output file path for the generated PPM.");

    po::options_description filterOptions("Generic Filtering Options");
    filterOptions.add_options()
        ("radius,r", po::value<double>(), "Search radius. Defaults to value "
            "calculated from estimated layer thickness.")
        ("interval,i", po::value<double>()->default_value(1.0),
            "Sampling interval")
        ("direction,d", po::value<int>()->default_value(0),
            "Sample Direction:\n"
                "  0 = Omni\n"
                "  1 = Positive\n"
                "  2 = Negative");

    po::options_description all("Usage");
    all.add(required).add(filterOptions);
    // clang-format on

    // Parse the cmd line
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

    // Get the parsed options
    fs::path volpkgPath = fs::canonical(parsed["volpkg"].as<std::string>());
    auto segID = parsed["seg"].as<std::string>();

    // Check for output file
    auto outputPath = fs::canonical(parsed["output-dir"].as<std::string>());
    if (!fs::is_directory(outputPath) || !fs::exists(outputPath)) {
        std::cerr << "Provided output path is not a directory or does not exist"
                  << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the volume package /////
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.getVersion() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion()
                  << " but this program requires version "
                  << VOLPKG_SUPPORTED_VERSION << "." << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    if (parsed.count("volume")) {
        volume = vpkg.volume(parsed["volume"].as<std::string>());
    } else {
        volume = vpkg.volume();
    }
    double cacheBytes = 0.75 * SystemMemorySize();
    volume->setCacheMemoryInBytes(static_cast<size_t>(cacheBytes));

    ///// Get some post-vpkg loading command line arguments /////
    // Get the texturing radius. If not specified, default to a radius
    // defined by the estimated thickness of the layer
    double radius;
    if (parsed.count("radius")) {
        radius = parsed["radius"].as<double>();
    } else {
        radius = vpkg.getMaterialThickness() / volume->voxelSize();
    }

    auto interval = parsed["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed["direction"].as<int>());

    ///// Load and resample the segmentation /////
    auto seg = vpkg.segmentation(segID);

    // Mesh the point cloud
    vc::meshing::OrderedPointSetMesher mesher;
    mesher.setPointSet(seg->getPointSet());
    auto input = mesher.compute();

    // Calculate sampling density
    auto voxelToMicron = std::pow(volume->voxelSize(), 2);
    auto area = vc::meshmath::SurfaceArea(input) * voxelToMicron * UM_TO_MM;
    auto vertCount = static_cast<uint16_t>(SAMPLING_DENSITY_FACTOR * area);
    vertCount = (vertCount < CLEANER_MIN_REQ_POINTS) ? CLEANER_MIN_REQ_POINTS
                                                     : vertCount;

    // Convert to polydata
    auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
    vc::meshing::ITK2VTK(input, vtkMesh);

    // Decimate using ACVD
    std::cout << "Resampling mesh..." << std::endl;
    auto acvdMesh = vtkSmartPointer<vtkPolyData>::New();
    vc::meshing::ACVD(vtkMesh, acvdMesh, vertCount);

    // Merge Duplicates
    // Note: This merging has to be the last in the process chain for some
    // really weird reason. - SP
    auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    cleaner->SetInputData(acvdMesh);
    cleaner->Update();

    auto itkACVD = vc::ITKMesh::New();
    vc::meshing::VTK2ITK(cleaner->GetOutput(), itkACVD);

    ///// ABF flattening /////
    std::cout << "Computing parameterization..." << std::endl;
    vc::texturing::AngleBasedFlattening abf(itkACVD);
    abf.compute();

    // Get UV map
    vc::UVMap uvMap = abf.getUVMap();
    auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<size_t>(std::ceil(uvMap.ratio().height));

    // Generate the PPM
    std::cout << "Generating PPM..." << std::endl;
    vc::texturing::PPMGenerator ppmGen;
    ppmGen.setMesh(itkACVD);
    ppmGen.setUVMap(uvMap);
    ppmGen.setDimensions(height, width);
    auto ppm = ppmGen.compute();

    vc::texturing::LayerTexture s;
    s.setVolume(volume);
    s.setPerPixelMap(ppm);
    s.setSamplingRadius(radius);
    s.setSamplingInterval(interval);
    s.setSamplingDirection(direction);
    auto texture = s.compute();

    // Write the layers
    std::cout << "Writing layers..." << std::endl;
    for (size_t i = 0; i < texture.numberOfImages(); ++i) {
        auto filepath = outputPath / (std::to_string(i) + ".png");
        cv::imwrite(filepath.string(), texture.image(i));
    }

    // Save the PPM
    if (parsed.count("output-ppm")) {
        std::cout << "Writing PPM..." << std::endl;
        fs::path ppmPath = parsed["output-ppm"].as<std::string>();
        vc::PerPixelMap::WritePPM(ppmPath, ppm);
    }

    return EXIT_SUCCESS;
}  // end main
