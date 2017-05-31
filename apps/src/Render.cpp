// render.cpp
// Abigail Coleman Feb. 2015

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vtkCleanPolyData.h>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/external/GetMemorySize.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/SmoothNormals.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 4;
// Number of vertices per square millimeter
static constexpr double SAMPLING_DENSITY_FACTOR = 50;
// Square Micron to square millimeter conversion factor
static constexpr double UM_TO_MM = 0.001 * 0.001;
// Min. number of points required to do flattening
static constexpr uint16_t CLEANER_MIN_REQ_POINTS = 100;

enum class Method { Composite = 0, Intersection, Integral };

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("seg,s", po::value<std::string>()->required(), "Segmentation ID")
        ("method,m", po::value<int>()->default_value(0),
            "Texturing method: \n"
                "  0 = Composite\n"
                "  1 = Intersection\n"
                "  2 = Integral")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: First volume.")
        ("output-file,o", po::value<std::string>(),
            "Output file path. If not specified, the file will be saved to the "
            "volume package.")
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

    po::options_description compositeOptions("Composite Texture Options");
    compositeOptions.add_options()
        ("filter,f", po::value<int>()->default_value(1),
            "Filter:\n"
                "  0 = Minimum\n"
                "  1 = Maximum\n"
                "  2 = Median\n"
                "  3 = Mean\n"
                "  4 = Median w/ Averaging");

    po::options_description integralOptions("Integral Texture Options");
    integralOptions.add_options()
        ("weight,w", po::value<int>()->default_value(2),
            "Value weighting:\n"
                "  0 = Favor the + normal direction\n"
                "  1 = Favor the - normal direction\n"
                "  2 = No weighting");

    po::options_description all("Usage");
    all.add(required).add(filterOptions).add(compositeOptions).add(integralOptions);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 5) {
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

    // Get the parsed options
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    auto segID = parsed["seg"].as<std::string>();
    Method method = static_cast<Method>(parsed["method"].as<int>());

    // Check for output file
    fs::path outputPath;
    if (parsed.count("output-file")) {
        outputPath = parsed["output-file"].as<std::string>();
        if (fs::exists(fs::canonical(outputPath.parent_path()))) {
            outputPath = fs::canonical(outputPath.parent_path()).string() +
                         "/" + outputPath.filename().string();
        } else {
            std::cerr << "ERROR: Cannot write to provided output file. "
                         "Output directory does not exist."
                      << std::endl;
        }
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
    auto filter = static_cast<vc::texturing::CompositeTexture::Filter>(
        parsed["filter"].as<int>());
    auto weight = static_cast<vc::texturing::IntegralTexture::Weight>(
        parsed["weight"].as<int>());

    ///// Load and resample the segmentation /////
    vpkg.setActiveSegmentation(segID);
    fs::path meshName = vpkg.getMeshPath();

    // try to convert the ply to an ITK mesh
    vc::io::PLYReader reader(meshName);
    try {
        reader.read();
    } catch (vc::IOException e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    auto input = reader.getMesh();

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

    ///// Generate texture /////
    vc::Texture texture;
    std::cout << "Generating Texture..." << std::endl;
    if (method == Method::Intersection) {
        vc::texturing::IntersectionTexture textureGen;
        textureGen.setVolume(volume);
        textureGen.setPerPixelMap(ppm);
        texture = textureGen.compute();
    }

    else if (method == Method::Composite) {
        vc::texturing::CompositeTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(volume);
        textureGen.setFilter(filter);
        textureGen.setSamplingRadius(radius);
        textureGen.setSamplingInterval(interval);
        textureGen.setSamplingDirection(direction);
        texture = textureGen.compute();
    }

    else if (method == Method::Integral) {
        vc::texturing::IntegralTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(volume);
        textureGen.setSamplingRadius(radius);
        textureGen.setSamplingInterval(interval);
        textureGen.setSamplingDirection(direction);
        textureGen.setWeight(weight);
        texture = textureGen.compute();
    }

    // Save rendering
    vc::Rendering rendering;
    rendering.setTexture(texture);
    rendering.setMesh(itkACVD);

    if (outputPath.extension() == ".PLY" || outputPath.extension() == ".ply") {
        std::cout << "Writing to PLY..." << std::endl;
        vc::io::PLYWriter writer(
            outputPath.string(), itkACVD, rendering.getTexture());
        writer.write();
    } else if (
        outputPath.extension() == ".OBJ" || outputPath.extension() == ".obj") {
        std::cout << "Writing to OBJ..." << std::endl;
        vc::io::OBJWriter writer;
        writer.setMesh(itkACVD);
        writer.setRendering(rendering);
        writer.setPath(outputPath.string());
        writer.write();
    } else if (
        outputPath.extension() == ".PNG" || outputPath.extension() == ".png") {
        std::cout << "Writing to PNG..." << std::endl;
        cv::imwrite(outputPath.string(), rendering.getTexture().image(0));
    } else {
        std::cout << "Writing to Volume Package..." << std::endl;
        vpkg.saveMesh(itkACVD, rendering.getTexture());
    }

    // Save the PPM
    if (parsed.count("output-ppm")) {
        std::cout << "Writing PPM..." << std::endl;
        fs::path ppmPath = parsed["output-ppm"].as<std::string>();
        vc::PerPixelMap::WritePPM(ppmPath, ppm);
    }

    return EXIT_SUCCESS;
}  // end main
