// render.cpp
// Abigail Coleman Feb. 2015

#include <iostream>

#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/app_support/GeneralOptions.hpp"
#include "vc/app_support/GetMemorySize.hpp"
#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/apps/render/RenderIO.hpp"
#include "vc/apps/render/RenderTexturing.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"
#include "vc/texturing/ThicknessTexture.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vct = volcart::texturing;

// Volpkg version required by this app
static constexpr int VOLPKG_MIN_VERSION = 6;

// Globals
po::variables_map parsed_;
/** Unused globals that need to be here */
vc::VolumePkg::Pointer vpkg_;
vc::Segmentation::Pointer seg_;
vc::Volume::Pointer volume_;
vc::UVMap parsedUVMap_;
/** End unused globals */

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description ioOpts("Input/Output Options");
    ioOpts.add_options()
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: First volume.")
        ("output-file,o", po::value<std::string>()->required(),
            "Output image file path.")
        ("tiff-floating-point", "When outputting to the TIFF format, save a "
            "floating-point image.");

    po::options_description all("Usage");
    all.add(GetGeneralOpts())
            .add(ioOpts)
            .add(GetFilteringOpts())
            .add(GetCompositeOpts())
            .add(GetIntegralOpts())
            .add(GetThicknessOpts());
    // clang-format on

    // Parse the cmd line
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed_);

    // Show the help message
    if (parsed_.count("help") || argc < 5) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed_);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get the parsed_ options
    fs::path volpkgPath = parsed_["volpkg"].as<std::string>();
    fs::path inputPPMPath = parsed_["ppm"].as<std::string>();
    Method method = static_cast<Method>(parsed_["method"].as<int>());
    fs::path outputPath = parsed_["output-file"].as<std::string>();

    ///// Load the volume package /////
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.version() < VOLPKG_MIN_VERSION) {
        vc::Logger()->error(
            "Volume Package is version {} but this program requires version "
            "{}+. ",
            vpkg.version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    try {
        if (parsed_.count("volume")) {
            volume = vpkg.volume(parsed_["volume"].as<std::string>());
        } else {
            volume = vpkg.volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct."
                  << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Set the cache size
    size_t cacheBytes;
    if (parsed_.count("cache-memory-limit")) {
        auto cacheSizeOpt = parsed_["cache-memory-limit"].as<std::string>();
        cacheBytes = vc::MemorySizeStringParser(cacheSizeOpt);
    } else {
        cacheBytes = SystemMemorySize() / 2;
    }
    volume->setCacheMemoryInBytes(cacheBytes);
    std::cout << "Volume Cache :: ";
    std::cout << "Capacity: " << volume->getCacheCapacity() << " || ";
    std::cout << "Size: " << vc::BytesToMemorySizeString(cacheBytes);
    std::cout << std::endl;

    ///// Get some post-vpkg loading command line arguments /////
    // Get the texturing radius. If not specified, default to a radius
    // defined by the estimated thickness of the layer
    cv::Vec3d radius{0, 0, 0};
    if (parsed_.count("radius")) {
        radius[0] = parsed_["radius"].as<double>();
    } else {
        radius = vpkg.materialThickness() / 2 / volume->voxelSize();
    }
    radius[1] = radius[2] = std::abs(std::sqrt(radius[0]));

    auto interval = parsed_["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed_["direction"].as<int>());
    auto shape = static_cast<Shape>(parsed_["neighborhood-shape"].as<int>());

    ///// Composite options /////
    auto filter =
        static_cast<vct::CompositeTexture::Filter>(parsed_["filter"].as<int>());

    ///// Integral options /////
    auto weightType = static_cast<vct::IntegralTexture::WeightMethod>(
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

    ///// Thickness options /////
    fs::path maskPath;
    if (parsed_.count("volume-mask") > 0) {
        maskPath = parsed_["volume-mask"].as<std::string>();
    }
    auto normalize = parsed_["normalize-output"].as<bool>();

    // Read the ppm
    std::cout << "Loading PPM..." << std::endl;
    auto ppm = vc::PerPixelMap::New(vc::PerPixelMap::ReadPPM(inputPPMPath));

    ///// Setup Neighborhood /////
    vc::NeighborhoodGenerator::Pointer generator;
    if (shape == Shape::Line) {
        auto line = vc::LineGenerator::New();
        generator = std::static_pointer_cast<vc::NeighborhoodGenerator>(line);
    } else {
        auto cube = vc::CuboidGenerator::New();
        generator = std::static_pointer_cast<vc::NeighborhoodGenerator>(cube);
    }
    generator->setSamplingRadius(radius);
    generator->setSamplingInterval(interval);
    generator->setSamplingDirection(direction);

    ///// Generate texture /////
    std::cout << "Generating Texture..." << std::endl;

    // Report selected generic options
    std::cout << "Neighborhood Parameters :: ";
    if (method == Method::Intersection) {
        std::cout << "Intersection";
    } else if (method == Method::Thickness) {
        std::cout << "Thickness || ";
        std::cout << "Sampling Interval: " << interval << " || ";
        std::cout << "Normalize Output: " << std::boolalpha << normalize;
    } else {
        std::cout << "Shape: ";
        if (shape == Shape::Line) {
            std::cout << "Line || ";
        } else {
            std::cout << "Cuboid || ";
        }
        std::cout << "Radius: " << radius << " || ";
        std::cout << "Sampling Interval: " << interval << " || ";
        std::cout << "Direction: ";
        if (direction == vc::Direction::Positive) {
            std::cout << "Positive";
        } else if (direction == vc::Direction::Negative) {
            std::cout << "Negative";
        } else {
            std::cout << "Both";
        }
    }
    std::cout << std::endl;

    vct::TexturingAlgorithm::Pointer textureGen;
    if (method == Method::Intersection) {
        auto intersect = vct::IntersectionTexture::New();
        intersect->setVolume(volume);
        intersect->setPerPixelMap(ppm);
        textureGen = intersect;
    }

    else if (method == Method::Composite) {
        auto composite = vct::CompositeTexture::New();
        composite->setPerPixelMap(ppm);
        composite->setVolume(volume);
        composite->setFilter(filter);
        composite->setGenerator(generator);
        textureGen = composite;
    }

    else if (method == Method::Integral) {
        auto integral = vct::IntegralTexture::New();
        integral->setPerPixelMap(ppm);
        integral->setVolume(volume);
        integral->setGenerator(generator);
        integral->setWeightMethod(weightType);
        integral->setLinearWeightDirection(weightDirection);
        integral->setExponentialDiffExponent(weightExponent);
        integral->setExponentialDiffBaseMethod(expoDiffBaseMethod);
        integral->setExponentialDiffBaseValue(expoDiffBase);
        integral->setClampValuesToMax(clampToMax);
        if (clampToMax) {
            integral->setClampMax(parsed_["clamp-to-max"].as<uint16_t>());
        }
        textureGen = integral;
    }

    else if (method == Method::Thickness) {
        // Load mask
        if (maskPath.empty()) {
            std::cerr << "ERROR: Selected Thickness texturing, but did not "
                         "provide volume mask path."
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::cout << "Loading volume mask..." << std::endl;
        auto pts = vc::PointSetIO<cv::Vec3i>::ReadPointSet(maskPath);
        auto mask = vc::VolumetricMask::New(pts);

        auto thickness = vct::ThicknessTexture::New();
        thickness->setPerPixelMap(ppm);
        thickness->setVolumetricMask(mask);
        thickness->setNormalizeOutput(normalize);
        textureGen = thickness;
    }
    if (parsed_["progress"].as<bool>()) {
        vc::ReportProgress(*textureGen, "Texturing:");
    } else {
        std::cout << "Texturing..." << std::endl;
    }

    auto texture = textureGen->compute();

    // Write the output
    SaveOutput(outputPath, nullptr, nullptr, texture);

    return EXIT_SUCCESS;
}  // end main
