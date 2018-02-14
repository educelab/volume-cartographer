// render.cpp

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vtkCleanPolyData.h>
#include <vtkSmoothPolyDataFilter.h>

#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/external/GetMemorySize.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/CalculateNormals.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;
namespace vct = volcart::texturing;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 5;
// Square Micron to square millimeter conversion factor
static constexpr double UM_TO_MM = 0.001 * 0.001;
// Min. number of points required to do flattening
static constexpr uint16_t CLEANER_MIN_REQ_POINTS = 100;

enum class Method { Composite = 0, Intersection, Integral };
enum class Smooth { Off = 0, Before, After, Both };

// Globals
po::variables_map parsed_;
vc::VolumePkg::Pointer vpkg_;
vc::Segmentation::Pointer seg_;
vc::Volume::Pointer volume_;

// File loading
vc::ITKMesh::Pointer loadSegmentation(const vc::Segmentation::Identifier& id);
vc::ITKMesh::Pointer loadMeshFile(const fs::path& p);

// Mesh resampling/smoothing
vc::ITKMesh::Pointer resampleMesh(vc::ITKMesh::Pointer m, Smooth smooth);

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("seg,s", po::value<std::string>(), "Segmentation ID")
        ("input-mesh", po::value<std::string>(), "Path to input OBJ or PLY")
        ("method,m", po::value<int>()->default_value(0),
            "Texturing method: \n"
                "  0 = Composite\n"
                "  1 = Intersection\n"
                "  2 = Integral")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: Segmentation's associated "
            "volume or the first volume in the volume package.")
        ("output-file,o", po::value<std::string>(),
            "Output file path. If not specified, an OBJ file and texture image "
            "will be placed in the current working directory.")
        ("output-ppm", po::value<std::string>(),
            "Output file path for the generated PPM.");

    po::options_description meshOptions("Mesh Filtering Options");
    meshOptions.add_options()
        ("enable-mesh-resampling", "Enable ACVD mesh resampling. Automatically "
            "enabled if the input is a Segmentation")
        ("mesh-resample-factor", po::value<double>()->default_value(50),
            "Roughly, the number of vertices per square millimeter in the "
            "output mesh")
        ("mesh-resample-keep-vcount", "If enabled, mesh resampling will "
            "attempt to maintain the number of vertices in the input mesh. "
            "Overrides the value set by --mesh-resample-factor.")
        ("mesh-resample-smoothing", po::value<int>()->default_value(0),
            "Smoothing Options:\n"
                "  0 = Off\n"
                "  1 = Before mesh resampling\n"
                "  2 = After mesh resampling\n"
                "  3 = Both before and after mesh resampling");

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
        ("weight-type,w", po::value<int>()->default_value(0),
            "Weight Type:\n"
                "  0 = None\n"
                "  1 = Linear\n"
                "  2 = Exponential Difference")
        ("linear-weight-direction", po::value<int>()->default_value(0),
            "Linear Weight Direction:\n"
                "  0 = Favor the + normal direction\n"
                "  1 = Favor the - normal direction")
        ("expodiff-exponent", po::value<int>()->default_value(2), "Exponent "
            "applied to the absolute difference values.")
        ("expodiff-base-method", po::value<int>()->default_value(0),
            "Exponential Difference Base Calculation Method:\n"
                "  0 = Mean\n"
                "  1 = Mode\n"
                "  2 = Manually specified")
        ("expodiff-base", po::value<double>()->default_value(0.0), "If the "
            "base calculation method is set to Manual, the value from which "
            "voxel values are differenced.")
        ("clamp-to-max", po::value<uint16_t>(), "Clamp values to the specified "
            "maximum.");

    po::options_description all("Usage");
    all.add(required).add(meshOptions).add(filterOptions).add(compositeOptions).add(integralOptions);
    // clang-format on

    // Parse the cmd line
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed_);

    // Show the help message
    if (parsed_.count("help") || argc < 5) {
        std::cerr << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed_);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get the parsed options
    fs::path volpkgPath = parsed_["volpkg"].as<std::string>();

    Method method = static_cast<Method>(parsed_["method"].as<int>());
    Smooth smooth =
        static_cast<Smooth>(parsed_["mesh-resample-smoothing"].as<int>());

    ///// Load the volume package /////
    vpkg_ = vc::VolumePkg::New(volpkgPath);
    if (vpkg_->version() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume Package is version " << vpkg_->version()
                  << " but this program requires version "
                  << VOLPKG_SUPPORTED_VERSION << "." << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the Segmentation or the Mesh file /////
    std::string filenameBase;
    vc::ITKMesh::Pointer input;
    bool loadSeg = parsed_.count("seg") > 0;
    bool loadMesh = parsed_.count("input-mesh") > 0;
    if (!loadSeg && !loadMesh) {
        std::cerr << "ERROR: Did not provide required flag Segmentation ID "
                     "(--seg) or mesh path (--input-mesh)"
                  << std::endl;
        return EXIT_FAILURE;
    } else if (loadSeg && loadMesh) {
        std::cerr << "ERROR: Provided Segmentation ID and mesh path as input. "
                     "Only one input is supported."
                  << std::endl;
        return EXIT_FAILURE;
    }

    if (loadSeg) {
        auto segID = parsed_["seg"].as<std::string>();
        input = loadSegmentation(segID);
        filenameBase = segID;
    } else {
        fs::path meshPath = parsed_["input-mesh"].as<std::string>();
        input = loadMeshFile(meshPath);
        filenameBase = meshPath.stem().string();
    }

    // Setup the output file
    fs::path outputPath;
    if (parsed_.count("output-file")) {
        outputPath = parsed_["output-file"].as<std::string>();
    } else {
        outputPath = filenameBase + "_render.obj";
    }

    ///// Load the Volume /////
    try {
        if (parsed_.count("volume")) {
            volume_ = vpkg_->volume(parsed_["volume"].as<std::string>());
        } else if (loadSeg && seg_->hasVolumeID()) {
            volume_ = vpkg_->volume(seg_->getVolumeID());
        } else {
            volume_ = vpkg_->volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct."
                  << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    double cacheBytes = 0.75 * SystemMemorySize();
    volume_->setCacheMemoryInBytes(static_cast<size_t>(cacheBytes));

    ///// Get some post-vpkg loading command line arguments /////
    // Get the texturing radius. If not specified, default to a radius
    // defined by the estimated thickness of the layer
    double radius;
    if (parsed_.count("radius")) {
        radius = parsed_["radius"].as<double>();
    } else {
        radius = vpkg_->materialThickness() / volume_->voxelSize();
    }

    // Generic texturing options
    auto interval = parsed_["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed_["direction"].as<int>());
    auto filter = static_cast<vc::texturing::CompositeTexture::Filter>(
        parsed_["filter"].as<int>());

    // Integral options
    auto weightType = static_cast<vct::IntegralTexture::WeightType>(
        parsed_["weight-type"].as<int>());
    auto weightDirection =
        static_cast<vct::IntegralTexture::LinearWeightDirection>(
            parsed_["linear-weight-direction"].as<int>());
    auto weightExponent = parsed_["expodiff-exponent"].as<int>();
    auto expoDiffBaseMethod =
        static_cast<vct::IntegralTexture::ExpoDiffBaseMethod>(
            parsed_["expodiff-base-method"].as<int>());
    auto expoDiffBase = parsed_["expodiff-base"].as<double>();
    auto clampToMax = parsed_.count("clamp-to-max") > 0;

    ///// Resample and smooth the mesh /////
    if (loadSeg || parsed_.count("enable-mesh-resampling")) {
        input = resampleMesh(input, smooth);
    }

    ///// ABF flattening /////
    std::cout << "Computing parameterization..." << std::endl;
    vc::texturing::AngleBasedFlattening abf(input);
    abf.compute();

    // Get UV map
    vc::UVMap uvMap = abf.getUVMap();
    auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<size_t>(std::ceil(uvMap.ratio().height));

    // Generate the PPM
    std::cout << "Generating PPM (" << width << "x" << height << ")...\n";
    vc::texturing::PPMGenerator ppmGen;
    ppmGen.setMesh(input);
    ppmGen.setUVMap(uvMap);
    ppmGen.setDimensions(height, width);
    auto ppm = ppmGen.compute();

    ///// Generate texture /////
    vc::Texture texture;
    std::cout << "Generating Texture..." << std::endl;
    if (method == Method::Intersection) {
        vc::texturing::IntersectionTexture textureGen;
        textureGen.setVolume(volume_);
        textureGen.setPerPixelMap(ppm);
        texture = textureGen.compute();
    }

    else if (method == Method::Composite) {
        vc::texturing::CompositeTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(volume_);
        textureGen.setFilter(filter);
        textureGen.setSamplingRadius(radius);
        textureGen.setSamplingInterval(interval);
        textureGen.setSamplingDirection(direction);
        texture = textureGen.compute();
    }

    else if (method == Method::Integral) {
        vc::texturing::IntegralTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(volume_);
        textureGen.setSamplingRadius(radius);
        textureGen.setSamplingInterval(interval);
        textureGen.setSamplingDirection(direction);
        textureGen.setWeightType(weightType);
        textureGen.setLinearWeightDirection(weightDirection);
        textureGen.setExponentialDiffExponent(weightExponent);
        textureGen.setExponentialDiffBaseMethod(expoDiffBaseMethod);
        textureGen.setExponentialDiffBase(expoDiffBase);
        textureGen.setClampValuesToMax(clampToMax);
        if (clampToMax) {
            textureGen.setClampMax(parsed_["clamp-to-max"].as<uint16_t>());
        }
        texture = textureGen.compute();
    }

    if (outputPath.extension() == ".PLY" || outputPath.extension() == ".ply") {
        std::cout << "Writing to PLY..." << std::endl;
        vc::io::PLYWriter writer(outputPath.string(), input, texture);
        writer.write();
    } else if (
        outputPath.extension() == ".PNG" || outputPath.extension() == ".png") {
        std::cout << "Writing to PNG..." << std::endl;
        cv::imwrite(outputPath.string(), texture.image(0));
    } else {
        std::cout << "Writing to OBJ..." << std::endl;
        vc::io::OBJWriter writer;
        writer.setMesh(input);
        writer.setUVMap(uvMap);
        writer.setTexture(texture.image(0));
        writer.setPath(outputPath.string());
        writer.write();
    }

    // Save the PPM
    if (parsed_.count("output-ppm")) {
        std::cout << "Writing PPM..." << std::endl;
        fs::path ppmPath = parsed_["output-ppm"].as<std::string>();
        vc::PerPixelMap::WritePPM(ppmPath, ppm);
    }

    return EXIT_SUCCESS;
}  // end main

vc::ITKMesh::Pointer loadSegmentation(const vc::Segmentation::Identifier& id)
{
    try {
        seg_ = vpkg_->segmentation(id);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load segmentation. ";
        std::cerr << "Please check the provided ID: " << id << std::endl;
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    // Load the point cloud
    std::cout << "Loading segmentation..." << std::endl;
    auto points = seg_->getPointSet();

    // Mesh the point cloud
    std::cout << "Meshing point set..." << std::endl;
    vcm::OrderedPointSetMesher mesher;
    mesher.setPointSet(points);
    return mesher.compute();
}

vc::ITKMesh::Pointer loadMeshFile(const fs::path& p)
{
    std::cout << "Loading mesh..." << std::endl;
    // OBJs
    if (vc::io::FileExtensionFilter(p, {"obj"})) {
        vc::io::OBJReader r;
        r.setPath(p);
        return r.read();
    }

    // PLYs
    else if (vc::io::FileExtensionFilter(p, {"ply"})) {
        vc::io::PLYReader r(p);
        return r.read();
    }

    // Can't load file
    else {
        std::cerr << "ERROR: Mesh file not of supported type: ";
        std::cerr << p << std::endl;
        exit(EXIT_FAILURE);
    }
}

vc::ITKMesh::Pointer resampleMesh(vc::ITKMesh::Pointer m, Smooth smooth)
{
    ///// Resample the segmentation /////
    // Calculate sampling density
    uint16_t vertCount{CLEANER_MIN_REQ_POINTS};
    auto voxelToMicron = std::pow(volume_->voxelSize(), 2);
    auto area = vc::meshmath::SurfaceArea(m) * voxelToMicron * UM_TO_MM;
    auto currentDensityFactor = m->GetNumberOfPoints() / area;
    double newDensityFactor;
    if (parsed_.count("mesh-resample-keep-vcount")) {
        newDensityFactor = currentDensityFactor;
        vertCount = static_cast<uint16_t>(m->GetNumberOfPoints());
    } else {
        newDensityFactor = parsed_["mesh-resample-factor"].as<double>();
        vertCount = static_cast<uint16_t>(newDensityFactor * area);
    }

    vertCount = (vertCount < CLEANER_MIN_REQ_POINTS) ? CLEANER_MIN_REQ_POINTS
                                                     : vertCount;
    // Convert to polydata
    auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
    vcm::ITK2VTK(m, vtkMesh);

    // Pre-Smooth
    if (smooth == Smooth::Both || smooth == Smooth::Before) {
        std::cout << "Smoothing mesh..." << std::endl;
        auto vtkSmoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        vtkSmoother->SetInputData(vtkMesh);
        vtkSmoother->Update();
        vtkMesh = vtkSmoother->GetOutput();
    }

    // Decimate using ACVD
    std::cout << "Resampling mesh (Density: " << currentDensityFactor << " -> "
              << newDensityFactor << ")..." << std::endl;
    auto acvdMesh = vtkSmartPointer<vtkPolyData>::New();
    vcm::ACVD(vtkMesh, acvdMesh, vertCount);

    // Merge Duplicates
    // Note: This merging has to be the last in the process chain for some
    // really weird reason. - SP
    auto cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    cleaner->SetInputData(acvdMesh);
    cleaner->Update();
    vtkMesh = cleaner->GetOutput();

    // Post-Smooth
    if (smooth == Smooth::Both || smooth == Smooth::After) {
        std::cout << "Smoothing mesh..." << std::endl;
        auto vtkSmoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        vtkSmoother->SetInputData(vtkMesh);
        vtkSmoother->Update();
        vtkMesh = vtkSmoother->GetOutput();
    }

    auto itkACVD = vc::ITKMesh::New();
    vcm::VTK2ITK(vtkMesh, itkACVD);

    // Make sure the normals are up-to-date if we've smoothed
    if (smooth == Smooth::Both || smooth == Smooth::Before ||
        smooth == Smooth::After) {
        vcm::CalculateNormals normals(itkACVD);
        itkACVD = normals.compute();
    }

    return itkACVD;
}