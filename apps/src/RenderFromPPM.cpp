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
#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"
#include "vc/texturing/ThicknessTexture.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vct = volcart::texturing;

// Volpkg version required by this app
static constexpr int VOLPKG_MIN_VERSION = 6;

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
        ("output-ppm", po::value<std::string>(), "Save a new PPM to the given "
            "path.")
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
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
        std::cout << all << '\n';
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Set logging level
    auto logLevel = parsed["log-level"].as<std::string>();
    logging::SetLogLevel(logLevel);

    // Get the parsed options
    const fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    const fs::path inputPPMPath = parsed["ppm"].as<std::string>();
    const auto method = static_cast<Method>(parsed["method"].as<int>());
    const fs::path outputPath = parsed["output-file"].as<std::string>();

    ///// Load the volume package /////
    auto vpkg = VolumePkg::New(volpkgPath);
    if (vpkg->version() < VOLPKG_MIN_VERSION) {
        Logger()->error(
            "Volume Package is version {} but this program requires version "
            "{}+. ",
            vpkg->version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    Volume::Pointer volume;
    try {
        if (parsed.count("volume") > 0) {
            volume = vpkg->volume(parsed["volume"].as<std::string>());
        } else {
            volume = vpkg->volume();
        }
    } catch (const std::exception& e) {
        Logger()->error(
            "Cannot load volume. Please check that the "
            "Volume Package has volumes and that the volume ID is correct: "
            "{}",
            e.what());
        return EXIT_FAILURE;
    }

    // Set the cache size
    std::size_t cacheBytes{SystemMemorySize() / 2};
    if (parsed.count("cache-memory-limit") > 0) {
        auto cacheSizeOpt = parsed["cache-memory-limit"].as<std::string>();
        cacheBytes = MemorySizeStringParser(cacheSizeOpt);
    }
    volume->setCacheMemoryInBytes(cacheBytes);
    Logger()->info(
        "Volume Cache :: Capacity: {} || Size: {}", volume->getCacheCapacity(),
        BytesToMemorySizeString(cacheBytes));

    ///// Get some post-vpkg loading command line arguments /////
    // Get the texturing radius. If not specified, default to a radius
    // defined by the estimated thickness of the layer
    cv::Vec3d radius{0, 0, 0};
    if (parsed.count("radius") > 0) {
        auto parsedRadius = parsed["radius"].as<std::vector<double>>();
        radius[0] = parsedRadius[0];
        radius[1] = radius[2] = std::sqrt(std::abs(radius[0]));
        if (parsedRadius.size() >= 2) {
            radius[1] = parsedRadius[1];
        }
        if (parsedRadius.size() >= 3) {
            radius[2] = parsedRadius[2];
        }
    } else {
        radius = vpkg->materialThickness() / 2 / volume->voxelSize();
    }
    radius[1] = radius[2] = std::abs(std::sqrt(radius[0]));

    auto interval = parsed["interval"].as<double>();
    auto direction = static_cast<Direction>(parsed["direction"].as<int>());
    auto shape = static_cast<Shape>(parsed["neighborhood-shape"].as<int>());

    ///// Composite options /////
    auto filter =
        static_cast<vct::CompositeTexture::Filter>(parsed["filter"].as<int>());

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

    ///// Thickness options /////
    fs::path maskPath;
    if (parsed.count("volume-mask") > 0) {
        maskPath = parsed["volume-mask"].as<std::string>();
    }
    auto normalize = parsed["normalize-output"].as<bool>();

    // Read the ppm
    Logger()->info("Loading PPM...");
    auto ppm = PerPixelMap::New(std::move(PerPixelMap::ReadPPM(inputPPMPath)));

    ///// Transform the PPM /////
    if (parsed.count("transform") > 0) {
        // load the transform
        auto tfmId = parsed.at("transform").as<std::string>();
        Transform3D::Pointer tfm;
        if (vpkg->hasTransform(tfmId)) {
            tfm = vpkg->transform(tfmId);
        } else {
            tfm = Transform3D::Load(tfmId);
        }

        if (parsed.count("invert-transform") > 0) {
            if (tfm->invertible()) {
                tfm = tfm->invert();
            } else {
                Logger()->warn("Cannot invert transform. Using original.");
            }
        }

        Logger()->info("Applying transform...");
        ppm = ApplyTransform(ppm, tfm);
    }

    ///// Setup Neighborhood /////
    Logger()->debug("Setting up generator...");
    NeighborhoodGenerator::Pointer generator;
    if (shape == Shape::Line) {
        auto line = LineGenerator::New();
        generator = std::static_pointer_cast<NeighborhoodGenerator>(line);
    } else {
        auto cube = CuboidGenerator::New();
        generator = std::static_pointer_cast<NeighborhoodGenerator>(cube);
    }
    generator->setSamplingRadius(radius);
    generator->setSamplingInterval(interval);
    generator->setSamplingDirection(direction);

    ///// Generate texture /////
    Logger()->info("Generating texture...");

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
        if (direction == Direction::Positive) {
            ss << "Positive";
        } else if (direction == Direction::Negative) {
            ss << "Negative";
        } else {
            ss << "Both";
        }
    }
    Logger()->info(ss.str());

    Logger()->debug("Setting up texturing algorithm...");
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
            integral->setClampMax(parsed["clamp-to-max"].as<uint16_t>());
        }
        textureGen = integral;
    }

    else if (method == Method::Thickness) {
        // Load mask
        if (maskPath.empty()) {
            Logger()->error(
                "Selected Thickness texturing, but did not "
                "provide volume mask path.");
            std::exit(EXIT_FAILURE);
        }
        Logger()->info("Loading volume mask...");
        auto pts = PointSetIO<cv::Vec3i>::ReadPointSet(maskPath);
        auto mask = VolumetricMask::New(pts);

        auto thickness = vct::ThicknessTexture::New();
        thickness->setPerPixelMap(ppm);
        thickness->setVolumetricMask(mask);
        thickness->setNormalizeOutput(normalize);
        textureGen = thickness;
    }

    if (parsed["progress"].as<bool>()) {
        ProgressConfig cfg;
        if (parsed.count("progress-interval") > 0) {
            cfg.interval = DurationFromString(
                parsed["progress-interval"].as<std::string>());
        }
        ReportProgress(*textureGen, "Texturing:", cfg);
        Logger()->debug("Texturing...");
    } else {
        Logger()->info("Texturing...");
    }

    Logger()->debug("Starting texturing algorithm...");
    auto texture = textureGen->compute();

    // Write the output
    Logger()->info("Writing output image...");
    WriteImage(outputPath, texture[0]);

    if (parsed.count("output-ppm") > 0) {
        Logger()->info("Writing output PPM...");
        const fs::path outputPPMPath = parsed["output-ppm"].as<std::string>();
        PerPixelMap::WritePPM(outputPPMPath, *ppm);
    }

    Logger()->info("Done.");
}  // end main
