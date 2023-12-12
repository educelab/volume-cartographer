#include <iostream>
#include <sstream>

#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/app_support/GeneralOptions.hpp"
#include "vc/app_support/GetMemorySize.hpp"
#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/apps/render/RenderTexturing.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/ImageIO.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Transforms.hpp"
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

namespace
{
auto GetTransformOpts() -> po::options_description
{
    // clang-format off
    po::options_description opts("Transform Options");
    opts.add_options()
        ("transform", po::value<std::string>(), "The ID of a transform in the "
            "VolumePkg or a path to a Transform3D .json file. If provided, "
            "perform coordinate transforms with the given transform.")
        ("invert-transform", "When provided, invert the transform.");
    // clang-format on

    return opts;
}
}  // namespace

auto main(int argc, char* argv[]) -> int
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
            .add(::GetTransformOpts())
            .add(GetFilteringOpts())
            .add(GetCompositeOpts())
            .add(GetIntegralOpts())
            .add(GetThicknessOpts());
    // clang-format on

    // Parse the cmd line
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed_);

    // Show the help message
    if (parsed_.count("help") > 0 || argc < 5) {
        std::cout << all << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed_);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // Get the parsed_ options
    const fs::path volpkgPath = parsed_["volpkg"].as<std::string>();
    const fs::path inputPPMPath = parsed_["ppm"].as<std::string>();
    const auto method = static_cast<Method>(parsed_["method"].as<int>());
    const fs::path outputPath = parsed_["output-file"].as<std::string>();

    ///// Load the volume package /////
    auto vpkg = vc::VolumePkg::New(volpkgPath);
    if (vpkg->version() < VOLPKG_MIN_VERSION) {
        vc::Logger()->error(
            "Volume Package is version {} but this program requires version "
            "{}+. ",
            vpkg->version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    try {
        if (parsed_.count("volume")) {
            volume = vpkg->volume(parsed_["volume"].as<std::string>());
        } else {
            volume = vpkg->volume();
        }
    } catch (const std::exception& e) {
        vc::Logger()->error(
            "Cannot load volume. Please check that the "
            "Volume Package has volumes and that the volume ID is correct: "
            "{}",
            e.what());
        return EXIT_FAILURE;
    }

    // Set the cache size
    std::size_t cacheBytes;
    if (parsed_.count("cache-memory-limit")) {
        auto cacheSizeOpt = parsed_["cache-memory-limit"].as<std::string>();
        cacheBytes = vc::MemorySizeStringParser(cacheSizeOpt);
    } else {
        cacheBytes = SystemMemorySize() / 2;
    }
    volume->setCacheMemoryInBytes(cacheBytes);
    vc::Logger()->info(
        "Volume Cache :: Capacity: {} || Size: {}", volume->getCacheCapacity(),
        vc::BytesToMemorySizeString(cacheBytes));

    ///// Get some post-vpkg loading command line arguments /////
    // Get the texturing radius. If not specified, default to a radius
    // defined by the estimated thickness of the layer
    cv::Vec3d radius{0, 0, 0};
    if (parsed_.count("radius") > 0) {
        radius[0] = parsed_["radius"].as<double>();
    } else {
        radius = vpkg->materialThickness() / 2 / volume->voxelSize();
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
    vc::Logger()->info("Loading PPM...");
    auto ppm =
        vc::PerPixelMap::New(std::move(vc::PerPixelMap::ReadPPM(inputPPMPath)));

    ///// Transform the PPM /////
    if (parsed_.count("transform") > 0) {
        // load the transform
        auto tfmId = parsed_.at("transform").as<std::string>();
        vc::Transform3D::Pointer tfm;
        if (vpkg->hasTransform(tfmId)) {
            tfm = vpkg->transform(tfmId);
        } else {
            tfm = vc::Transform3D::Load(tfmId);
        }

        if (parsed_.count("invert-transform") > 0) {
            if (tfm->invertible()) {
                tfm = tfm->invert();
            } else {
                vc::Logger()->warn("Cannot invert transform. Using original.");
            }
        }

        vc::Logger()->info("Applying transform...");
        ppm = vc::ApplyTransform(ppm, tfm);
    }

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
    vc::Logger()->info("Generating texture...");

    // Report selected generic options
    std::ostringstream ss;
    ss << "Neighborhood Parameters :: ";
    if (method == Method::Intersection) {
        ss << "Intersection";
    } else if (method == Method::Thickness) {
        ss << "Thickness || ";
        ss << "Sampling Interval: " << interval << " || ";
        ss << "Normalize Output: " << std::boolalpha << normalize;
    } else {
        ss << "Shape: ";
        if (shape == Shape::Line) {
            ss << "Line || ";
        } else {
            ss << "Cuboid || ";
        }
        ss << "Radius: " << radius << " || ";
        ss << "Sampling Interval: " << interval << " || ";
        ss << "Direction: ";
        if (direction == vc::Direction::Positive) {
            ss << "Positive";
        } else if (direction == vc::Direction::Negative) {
            ss << "Negative";
        } else {
            ss << "Both";
        }
    }
    vc::Logger()->info(ss.str());

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
            vc::Logger()->error(
                "Selected Thickness texturing, but did not "
                "provide volume mask path.");
            std::exit(EXIT_FAILURE);
        }
        vc::Logger()->info("Loading volume mask...");
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
        vc::Logger()->debug("Texturing...");
    } else {
        vc::Logger()->info("Texturing...");
    }

    auto texture = textureGen->compute();

    // Write the output
    vc::WriteImage(outputPath, texture[0]);

    vc::Logger()->info("Done.");
    return EXIT_SUCCESS;
}  // end main
