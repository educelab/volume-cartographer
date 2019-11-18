// render.cpp
// Abigail Coleman Feb. 2015

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/ImageConversion.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/external/GetMemorySize.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vct = volcart::texturing;

// File loading
auto ExtFilter = &vc::io::FileExtensionFilter;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;

enum class Method { Composite = 0, Intersection, Integral };
enum class Shape { Line = 0, Cuboid };

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("method,m", po::value<int>()->default_value(0),
            "Texturing method: \n"
                "  0 = Composite\n"
                "  1 = Intersection\n"
                "  2 = Integral")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: First volume.")
        ("output-file,o", po::value<std::string>()->required(),
            "Output image file path.")
        ("tiff-floating-point", "When outputting to the TIFF format, save a "
            "floating-point image.");

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
                "  2 = Negative")
        ("neighborhood-shape,n", po::value<int>()->default_value(0),
             "Neighborhood shape:\n"
                 "  0 = Linear\n"
                 "  1 = Cuboid");

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

    po::options_description performanceOptions("Performance Options");
    performanceOptions.add_options()
        ("cache-memory-limit", po::value<std::string>(), "Maximum size of the "
            "slice cache in bytes. Accepts the suffixes: (K|M|G|T)(B). "
            "Default: 50% of the total system memory.");

    po::options_description all("Usage");
    all.add(required)
            .add(filterOptions)
            .add(compositeOptions)
            .add(integralOptions)
            .add(performanceOptions);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 5) {
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
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    fs::path inputPPMPath = parsed["ppm"].as<std::string>();
    Method method = static_cast<Method>(parsed["method"].as<int>());
    fs::path outputPath = parsed["output-file"].as<std::string>();

    ///// Load the volume package /////
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.version() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume package is version " << vpkg.version()
                  << " but this program requires version "
                  << VOLPKG_SUPPORTED_VERSION << "." << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    try {
        if (parsed.count("volume")) {
            volume = vpkg.volume(parsed["volume"].as<std::string>());
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
    if (parsed.count("cache-memory-limit")) {
        auto cacheSizeOpt = parsed["cache-memory-limit"].as<std::string>();
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
    if (parsed.count("radius")) {
        radius[0] = parsed["radius"].as<double>();
    } else {
        radius = vpkg.materialThickness() / 2 / volume->voxelSize();
    }
    radius[1] = radius[2] = std::abs(std::sqrt(radius[0]));

    auto interval = parsed["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed["direction"].as<int>());
    auto shape = static_cast<Shape>(parsed["neighborhood-shape"].as<int>());

    ///// Composite options /////
    auto filter = static_cast<vc::texturing::CompositeTexture::Filter>(
        parsed["filter"].as<int>());

    ///// Integral options /////
    auto weightType = static_cast<vct::IntegralTexture::WeightMethod>(
        parsed["weight-type"].as<int>());
    auto weightDirection =
        static_cast<vct::IntegralTexture::LinearWeightDirection>(
            parsed["linear-weight-direction"].as<int>());
    auto weightExponent = parsed["expodiff-exponent"].as<int>();
    auto expoDiffBaseMethod =
        static_cast<vct::IntegralTexture::ExpoDiffBaseMethod>(
            parsed["expodiff-base-method"].as<int>());
    auto expoDiffBase = parsed["expodiff-base"].as<double>();
    auto clampToMax = parsed.count("clamp-to-max") > 0;

    // Read the ppm
    std::cout << "Loading PPM..." << std::endl;
    auto ppm = vc::PerPixelMap::ReadPPM(inputPPMPath);

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
    vc::Texture texture;
    std::cout << "Generating Texture..." << std::endl;

    // Report selected generic options
    std::cout << "Neighborhood Parameters :: ";
    if (method == Method::Intersection) {
        std::cout << "Intersection";
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
        textureGen.setGenerator(generator);
        texture = textureGen.compute();
    }

    else if (method == Method::Integral) {
        vc::texturing::IntegralTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(volume);
        textureGen.setGenerator(generator);
        textureGen.setWeightMethod(weightType);
        textureGen.setLinearWeightDirection(weightDirection);
        textureGen.setExponentialDiffExponent(weightExponent);
        textureGen.setExponentialDiffBaseMethod(expoDiffBaseMethod);
        textureGen.setExponentialDiffBaseValue(expoDiffBase);
        textureGen.setClampValuesToMax(clampToMax);
        if (clampToMax) {
            textureGen.setClampMax(parsed["clamp-to-max"].as<uint16_t>());
        }
        texture = textureGen.compute();
    }

    // Convert to/from floating point
    auto requestTIFF = ExtFilter(outputPath, {"tiff", "tif"});
    auto requestFloat = parsed.count("tiff-floating-point") > 0;
    if (requestTIFF && requestFloat) {
        auto m = vc::QuantizeImage(texture.image(0), CV_32F);
        texture.setImage(0, m);
    } else {
        auto m = texture.image(0);
        if (m.depth() == CV_32F || m.depth() == CV_64F) {
            m = vc::QuantizeImage(m, CV_16U);
            texture.setImage(0, m);
        }
    }

    // Write the output
    if (requestTIFF && requestFloat) {
        std::cout << "Writing to floating-point TIF ..." << std::endl;
        vc::tiffio::WriteTIFF(outputPath, texture.image(0));
    } else if (ExtFilter(outputPath, {"png", "jpg", "jpeg", "tiff", "tif"})) {
        std::cout << "Writing to image..." << std::endl;
        cv::imwrite(outputPath.string(), texture.image(0));
    } else {
        std::cerr << "Unrecognized output format: " << outputPath.extension();
        std::cerr << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}  // end main
