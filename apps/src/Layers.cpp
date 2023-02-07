#include <fstream>
#include <iostream>
#include <sstream>

#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

#include "vc/app_support/GetMemorySize.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/ImageIO.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/core/util/String.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/meshing/SmoothNormals.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/LayerTexture.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

using volcart::enumerate;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;
// Number of vertices per square millimeter
static constexpr double SAMPLING_DENSITY_FACTOR = 50;
// Square Micron to square millimeter conversion factor
static constexpr double UM_TO_MM = 0.001 * 0.001;
// Min. number of points required to do flattening
static constexpr size_t CLEANER_MIN_REQ_POINTS = 100;

auto main(int argc, char* argv[]) -> int
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
            "Volume to use for texturing. Default: Segmentation's associated "
            "volume or the first volume in the volume package.")
        ("output-dir,o", po::value<std::string>()->required(),
            "Output directory for layer images.")
        ("image-format,f", po::value<std::string>()->default_value("png"),
            "Image format for layer images. Default: png")
        ("compression", po::value<int>(), "Image compression level");

    po::options_description filterOptions("Generic Filtering Options");
    filterOptions.add_options()
        ("radius,r", po::value<double>(), "Search radius. Defaults to value "
            "calculated from estimated layer thickness.")
        ("interval,i", po::value<double>()->default_value(1.0),
            "Sampling interval")
        ("direction,d", po::value<int>()->default_value(0),
            "Sample Direction:\n"
                " -1 = Negative\n"
                "  0 = Omni\n"
                "  1 = Positive");

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
        vc::Logger()->error(e.what());
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
    auto imgFmt = vc::to_lower_copy(parsed["image-format"].as<std::string>());
    vc::WriteImageOpts writeOpts;
    if (parsed.count("compression") > 0) {
        writeOpts.compression = parsed["compression"].as<int>();
    } else {
        // Default for tiff in this app: No compression
        if (imgFmt == "tif" or imgFmt == "tiff") {
            writeOpts.compression = 1;
        }
    }

    ///// Load the volume package /////
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.version() != VOLPKG_SUPPORTED_VERSION) {
        std::stringstream msg;
        msg << "Volume package is version " << vpkg.version()
            << " but this program requires version " << VOLPKG_SUPPORTED_VERSION
            << ".";
        vc::Logger()->error(msg.str());
        return EXIT_FAILURE;
    }

    ///// Load the segmentation /////
    vc::Segmentation::Pointer seg;
    try {
        seg = vpkg.segmentation(segID);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load segmentation. ";
        std::cerr << "Please check the provided ID: " << segID << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    vc::Volume::Identifier volID;

    if (parsed.count("volume")) {
        volID = parsed["volume"].as<std::string>();
    } else if (seg->hasVolumeID()) {
        volID = seg->getVolumeID();
    }

    try {
        if (!volID.empty()) {
            volume = vpkg.volume(volID);
        } else {
            volume = vpkg.volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct. "
                  << volID << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
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
        radius = vpkg.materialThickness() / volume->voxelSize();
    }

    auto interval = parsed["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed["direction"].as<int>());

    ///// Resample the segmentation /////
    // Mesh the point cloud
    vc::meshing::OrderedPointSetMesher mesher;
    mesher.setPointSet(seg->getPointSet());
    auto input = mesher.compute();

    // Calculate sampling density
    auto voxelToMicron = std::pow(volume->voxelSize(), 2);
    auto area = vc::meshmath::SurfaceArea(input) * voxelToMicron * UM_TO_MM;
    auto vertCount = static_cast<size_t>(SAMPLING_DENSITY_FACTOR * area);
    vertCount = (vertCount < CLEANER_MIN_REQ_POINTS) ? CLEANER_MIN_REQ_POINTS
                                                     : vertCount;

    // Decimate using ACVD
    std::cout << "Resampling mesh..." << std::endl;
    vc::meshing::ACVD resampler;
    resampler.setInputMesh(input);
    resampler.setNumberOfClusters(vertCount);
    auto itkACVD = resampler.compute();

    ///// ABF flattening /////
    std::cout << "Computing parameterization..." << std::endl;
    vc::texturing::AngleBasedFlattening abf;
    abf.setMesh(itkACVD);
    abf.compute();

    // Get UV map
    auto uvMap = abf.getUVMap();
    auto width = static_cast<size_t>(std::ceil(uvMap->ratio().width));
    auto height = static_cast<size_t>(std::ceil(uvMap->ratio().height));

    // Generate the PPM
    std::cout << "Generating PPM..." << std::endl;
    vc::texturing::PPMGenerator ppmGen;
    ppmGen.setMesh(itkACVD);
    ppmGen.setUVMap(uvMap);
    ppmGen.setDimensions(height, width);
    auto ppm = ppmGen.compute();

    // Setup line generator
    auto line = vc::LineGenerator::New();
    line->setSamplingRadius(radius);
    line->setSamplingInterval(interval);
    line->setSamplingDirection(direction);

    // Layer texture
    std::cout << "Generating layers..." << std::endl;
    vc::texturing::LayerTexture s;
    s.setVolume(volume);
    s.setPerPixelMap(ppm);
    s.setGenerator(line);

    auto texture = s.compute();

    // Write the layers
    std::cout << "Writing layers..." << std::endl;
    const int numChars =
        static_cast<int>(std::to_string(texture.size()).size());
    fs::path filepath;
    for (const auto [i, image] : enumerate(texture)) {
        auto fileName = vc::to_padded_string(i, numChars) + "." + imgFmt;
        filepath = outputPath / fileName;
        vc::WriteImage(filepath, image, writeOpts);
    }

    return EXIT_SUCCESS;
}  // end main
