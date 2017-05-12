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

enum class Method { Composite = 0, Intersection };

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
                 "  1 = Intersection\n")
        ("output-file,o", po::value<std::string>(),
            "Output file path. If not specified, the file will be saved to the "
            "volume package.");

    po::options_description compositeOptions("Composite Texture Options");
    compositeOptions.add_options()
        ("radius,r", po::value<double>(), "Search radius. Defaults to value "
            "calculated from estimated layer thickness.")
        ("interval,i", po::value<double>()->default_value(1.0),
            "Sampling interval")
        ("filter,f", po::value<int>()->default_value(1),
         "Filter:\n"
                 "  0 = Minimum\n"
                 "  1 = Maximum\n"
                 "  2 = Median\n"
                 "  3 = Mean\n"
                 "  4 = Median w/ Averaging\n")
        ("direction,d", po::value<int>()->default_value(0),
         "Sample Direction:\n"
                 "  0 = Omni\n"
                 "  1 = Positive\n"
                 "  2 = Negative\n");

    po::options_description all("Usage");
    all.add(required).add(compositeOptions);
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
    double cacheBytes = 0.75 * SystemMemorySize();
    vpkg.volume()->setCacheMemoryInBytes(static_cast<size_t>(cacheBytes));

    ///// Get some post-vpkg loading command line arguments /////
    // Get the texturing radius. If not specified, default to a radius
    // defined by the estimated thickness of the layer
    double radius;
    if (parsed.count("radius")) {
        radius = parsed["radius"].as<double>();
    } else {
        radius = vpkg.getMaterialThickness() / vpkg.volume()->voxelSize();
    }

    auto interval = parsed["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed["direction"].as<int>());
    auto filter = static_cast<vc::texturing::CompositeTexture::Filter>(
        parsed["filter"].as<int>());

    ///// Load and resample the segmentation /////
    vpkg.setActiveSegmentation(segID);
    fs::path meshName = vpkg.getMeshPath();

    // try to convert the ply to an ITK mesh
    volcart::io::PLYReader reader(meshName);
    try {
        reader.read();
    } catch (volcart::IOException e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    auto input = reader.getMesh();

    // Calculate sampling density
    auto voxelToMicron = std::pow(vpkg.volume()->voxelSize(), 2);
    auto area = vc::meshmath::SurfaceArea(input) * voxelToMicron * UM_TO_MM;
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
    auto Cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    Cleaner->SetInputData(acvdMesh);
    Cleaner->Update();

    auto itkACVD = volcart::ITKMesh::New();
    volcart::meshing::VTK2ITK(Cleaner->GetOutput(), itkACVD);

    ///// ABF flattening /////
    std::cout << "Computing parameterization..." << std::endl;
    volcart::texturing::AngleBasedFlattening abf(itkACVD);
    abf.compute();

    // Get UV map
    volcart::UVMap uvMap = abf.getUVMap();
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
    volcart::Texture texture;
    std::cout << "Generating Texture..." << std::endl;
    if (method == Method::Intersection) {
        vc::texturing::IntersectionTexture textureGen;
        textureGen.setVolume(vpkg.volume());
        textureGen.setPerPixelMap(ppm);
        texture = textureGen.compute();
    } else {
        vc::texturing::CompositeTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(vpkg.volume());
        textureGen.setFilter(filter);
        textureGen.setSamplingRadius(radius);
        textureGen.setSamplingInterval(interval);
        textureGen.setSamplingDirection(direction);
        texture = textureGen.compute();
    }

    // Setup rendering
    volcart::Rendering rendering;
    rendering.setTexture(texture);
    rendering.setMesh(itkACVD);

    if (outputPath.extension() == ".PLY" || outputPath.extension() == ".ply") {
        std::cout << "Writing to PLY..." << std::endl;
        volcart::io::PLYWriter writer(
            outputPath.string(), itkACVD, rendering.getTexture());
        writer.write();
    } else if (
        outputPath.extension() == ".OBJ" || outputPath.extension() == ".obj") {
        std::cout << "Writing to OBJ..." << std::endl;
        volcart::io::OBJWriter writer;
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

    return EXIT_SUCCESS;
}  // end main
