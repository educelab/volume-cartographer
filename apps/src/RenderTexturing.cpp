#include "apps/RenderTexturing.hpp"

#include <exception>
#include <memory>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/Color.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"
#include "vc/texturing/OrthographicProjectionFlattening.hpp"
#include "vc/texturing/PPMGenerator.hpp"
#include "vc/texturing/ScaleMarkerGenerator.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vct = volcart::texturing;

extern po::variables_map parsed_;
extern vc::VolumePkg::Pointer vpkg_;
extern vc::Volume::Pointer volume_;
extern vc::UVMap parsedUVMap_;

// PI
static constexpr double PI_CONST = 3.14159265358979323846264338327950288;
// Degrees to Radians
static constexpr double DEG_TO_RAD = PI_CONST / 180.0;

// Aliases
using Shading = vct::PPMGenerator::Shading;
using ScaleGenerator = vct::ScaleMarkerGenerator;

// Local functions
enum class ScaleColorOpt { White = 0, Black, Red, Green, Cyan };
vc::Color GetScaleColorOpt();

enum class FlatteningAlgorithm { ABF = 0, LSCM, Orthographic };

po::options_description GetUVOpts()
{
    // clang-format off
    po::options_description opts("Flattening & UV Options");
    opts.add_options()
        ("uv-algorithm", po::value<int>()->default_value(0),
            "Select the flattening algorithm:\n"
                "  0 = ABF\n"
                "  1 = LSCM\n"
                "  2 = Orthographic Projection")
        ("reuse-uv", "If input-mesh is specified, attempt to use its existing "
            "UV map instead of generating a new one.")
        ("uv-rotate", po::value<double>(), "Rotate the generated UV map by an "
            "angle in degrees.")
        ("uv-flip", po::value<int>(),
            "Flip the UV map along an axis. If uv-rotate is specified, flip is "
            "performed after rotation.\n"
            "Axis along which to flip:\n"
                "  0 = Vertical\n"
                "  1 = Horizontal\n"
                "  2 = Both")
        ("uv-plot", po::value<std::string>(), "Plot the UV map and save "
            "it to the provided image path.");
    // clang-format on

    return opts;
}

po::options_description GetFilteringOpts()
{
    // clang-format off
    po::options_description opts("Generic Texture Filtering Options");
    opts.add_options()
        ("method,m", po::value<int>()->default_value(0),
             "Texturing method: \n"
                 "  0 = Composite\n"
                 "  1 = Intersection\n"
                 "  2 = Integral")
        ("neighborhood-shape,n", po::value<int>()->default_value(0),
             "Neighborhood shape:\n"
                 "  0 = Linear\n"
                 "  1 = Cuboid")
        ("radius,r", po::value<double>(), "Search radius. Defaults to value "
            "calculated from estimated layer thickness.")
        ("interval,i", po::value<double>()->default_value(1.0),
            "Sampling interval")
        ("direction,d", po::value<int>()->default_value(0),
            "Sample Direction:\n"
                "  0 = Omni\n"
                "  1 = Positive\n"
                "  2 = Negative")
        ("shading", po::value<int>()->default_value(1),
            "Surface Normal Shading:\n"
                "  0 = Flat\n"
                "  1 = Smooth");
    // clang-format on

    return opts;
}

po::options_description GetCompositeOpts()
{
    // clang-format off
    po::options_description opts("Composite Texture Options");
    opts.add_options()
        ("filter,f", po::value<int>()->default_value(1),
            "Filter:\n"
                "  0 = Minimum\n"
                "  1 = Maximum\n"
                "  2 = Median\n"
                "  3 = Mean\n"
                "  4 = Median w/ Averaging");
    // clang-format on

    return opts;
}

po::options_description GetIntegralOpts()
{
    // clang-format off
    po::options_description opts("Integral Texture Options");
    opts.add_options()
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
    // clang-format on

    return opts;
}

po::options_description GetPostProcessOpts()
{
    // clang-format off
    po::options_description opts("Texture Post-Processing Options");
    opts.add_options()
        ("scale-marker", po::value<std::string>(),
            "Output path for texture image with scale marker")
        ("scale-marker-type", po::value<int>()->default_value(0),
            "Scale Options:\n"
                "  0 = Scale Bar (Centimeters)\n"
                "  1 = Scale Bar (Inches)\n"
                "  2 = Reference Image")
        ("scale-marker-color", po::value<int>()->default_value(0),
            "Scale bar color:\n"
                "  0 = White\n"
                "  1 = Black\n"
                "  2 = Red\n"
                "  3 = Green\n"
                "  4 = Cyan")
        ("scale-marker-ref-img", po::value<std::string>(),
            "Path to reference image to be used as scale marker")
        ("scale-marker-ref-pixsize", po::value<double>(),
            "Pixel size (in microns) for the reference image");
    // clang-format on

    return opts;
}

vc::UVMap FlattenMesh(const vc::ITKMesh::Pointer& mesh, bool resampled)
{
    vc::UVMap uvMap;
    if (parsed_.count("reuse-uv")) {
        if (!resampled) {
            uvMap = parsedUVMap_;
        } else {
            std::cerr << "Warning: 'reuse-uv' option provided, but input mesh "
                         "has been resampled. Ignoring existing UV map.\n";
        }
    }

    // If we don't have a valid UV map yet, make one
    if (uvMap.empty()) {
        std::cout << "Computing parameterization..." << std::endl;
        auto method =
            static_cast<FlatteningAlgorithm>(parsed_["uv-algorithm"].as<int>());

        // ABF and LSCM
        if (method == FlatteningAlgorithm::ABF ||
            method == FlatteningAlgorithm::LSCM) {
            vct::AngleBasedFlattening abf(mesh);
            abf.setUseABF(method == FlatteningAlgorithm::ABF);
            abf.compute();
            uvMap = abf.getUVMap();
        }
        // Orthographic
        else if (method == FlatteningAlgorithm::Orthographic) {
            vct::OrthographicProjectionFlattening ortho;
            ortho.setMesh(mesh);
            ortho.compute();
            uvMap = ortho.getUVMap();
        }
    }

    // Rotate
    if (parsed_.count("uv-rotate") > 0) {
        auto theta = parsed_["uv-rotate"].as<double>();
        std::cout << "Rotating UV map " << theta << " degrees..." << std::endl;
        theta *= DEG_TO_RAD;
        vc::UVMap::Rotate(uvMap, theta);
    }

    // Flip
    if (parsed_.count("uv-flip") > 0) {
        auto axis =
            static_cast<vc::UVMap::FlipAxis>(parsed_["uv-flip"].as<int>());
        std::cout << "Flipping UV map..." << std::endl;
        vc::UVMap::Flip(uvMap, axis);
    }

    // Plot the UV Map
    if (parsed_.count("uv-plot") > 0) {
        std::cout << "Saving UV plot..." << std::endl;
        fs::path uvPlotPath = parsed_["uv-plot"].as<std::string>();
        cv::imwrite(uvPlotPath.string(), vc::UVMap::Plot(uvMap));
    }

    return uvMap;
}

vc::Texture TextureMesh(
    const vc::ITKMesh::Pointer& mesh, const vc::UVMap& uvMap)
{
    ///// Get some post-vpkg loading command line arguments /////
    // Get the texturing radius. If not specified, default to a radius
    // defined by the estimated thickness of the layer
    cv::Vec3d radius{0, 0, 0};
    if (parsed_.count("radius")) {
        radius[0] = parsed_["radius"].as<double>();
    } else {
        radius[0] = vpkg_->materialThickness() / 2 / volume_->voxelSize();
    }
    radius[1] = radius[2] = std::abs(std::sqrt(radius[0]));

    // Generic texturing options
    Method method = static_cast<Method>(parsed_["method"].as<int>());
    auto interval = parsed_["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed_["direction"].as<int>());
    auto shape = static_cast<Shape>(parsed_["neighborhood-shape"].as<int>());
    auto shading = static_cast<Shading>(parsed_["shading"].as<int>());

    ///// Composite options /////
    auto filter = static_cast<vc::texturing::CompositeTexture::Filter>(
        parsed_["filter"].as<int>());

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

    ///// Generate the PPM /////
    auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<size_t>(std::ceil(uvMap.ratio().height));
    std::cout << "Generating PPM (" << width << "x" << height << ")...\n";
    vct::PPMGenerator ppmGen;
    ppmGen.setMesh(mesh);
    ppmGen.setUVMap(uvMap);
    ppmGen.setDimensions(height, width);
    ppmGen.setShading(shading);
    auto ppm = ppmGen.compute();

    // Save the PPM
    if (parsed_.count("output-ppm")) {
        std::cout << "Writing PPM..." << std::endl;
        fs::path ppmPath = parsed_["output-ppm"].as<std::string>();
        vc::PerPixelMap::WritePPM(ppmPath, ppm);
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
        vct::IntersectionTexture textureGen;
        textureGen.setVolume(volume_);
        textureGen.setPerPixelMap(ppm);
        texture = textureGen.compute();
    }

    else if (method == Method::Composite) {
        vct::CompositeTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(volume_);
        textureGen.setFilter(filter);
        textureGen.setGenerator(generator);
        texture = textureGen.compute();
    }

    else if (method == Method::Integral) {
        vct::IntegralTexture textureGen;
        textureGen.setPerPixelMap(ppm);
        textureGen.setVolume(volume_);
        textureGen.setGenerator(generator);
        textureGen.setWeightMethod(weightType);
        textureGen.setLinearWeightDirection(weightDirection);
        textureGen.setExponentialDiffExponent(weightExponent);
        textureGen.setExponentialDiffBaseMethod(expoDiffBaseMethod);
        textureGen.setExponentialDiffBaseValue(expoDiffBase);
        textureGen.setClampValuesToMax(clampToMax);
        if (clampToMax) {
            textureGen.setClampMax(parsed_["clamp-to-max"].as<uint16_t>());
        }
        texture = textureGen.compute();
    }

    return texture;
}

void RenderPostProcess(const vc::Texture& texture)
{
    ///// Scale Markers /////
    if (parsed_.count("scale-marker") > 0) {
        // Get scale generator type
        auto type = static_cast<ScaleGenerator::Type>(
            parsed_["scale-marker-type"].as<int>());

        // Setup scale generator with what we know
        ScaleGenerator scaleGen;
        scaleGen.setInputImage(texture.image(0));
        scaleGen.setInputImagePixelSize(volume_->voxelSize());
        scaleGen.setScaleType(type);
        scaleGen.setScaleBarColor(GetScaleColorOpt());

        // Load reference image if necessary
        auto haveRefOpts = parsed_.count("scale-marker-ref-img") > 0 &&
                           (parsed_.count("scale-marker-ref-pixsize") > 0);
        if (type == ScaleGenerator::Type::ReferenceImage && haveRefOpts) {
            // Load the reference image
            fs::path refImgPath =
                parsed_["scale-marker-ref-img"].as<std::string>();
            auto refImg = cv::imread(refImgPath.string(), cv::IMREAD_UNCHANGED);

            // Get the pixel size of the reference image
            auto refPixSize = parsed_["scale-marker-ref-pixsize"].as<double>();

            // Set the appropriate
            scaleGen.setReferenceImage(refImg);
            scaleGen.setReferenceImagePixelSize(refPixSize);
        }

        // Revert to default bar if want reference image but missing req. opts
        else if (type == ScaleGenerator::Type::ReferenceImage) {
            std::cerr << "Warning: Specified reference image scale marker but "
                         "missing required options. Falling back to default "
                         "scale marker."
                      << std::endl;
            scaleGen.setScaleType(ScaleGenerator::Type::Metric);
        }

        // Generate scale marker
        std::cout << "Generating scale markers..." << std::endl;
        cv::Mat result;
        try {
            result = scaleGen.compute();
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            // Skip the rest of this block
            return;
        }

        // Write to disk
        fs::path scalePath = parsed_["scale-marker"].as<std::string>();
        cv::imwrite(scalePath.string(), result);
    }
}

vc::Color GetScaleColorOpt()
{
    auto colorOpt =
        static_cast<ScaleColorOpt>(parsed_["scale-marker-color"].as<int>());
    switch (colorOpt) {
        case ScaleColorOpt::White:
            return vc::color::WHITE;
        case ScaleColorOpt::Black:
            return vc::color::BLACK;
        case ScaleColorOpt::Red:
            return vc::color::RED;
        case ScaleColorOpt::Green:
            return vc::color::GREEN;
        case ScaleColorOpt::Cyan:
            return vc::color::CYAN;
    }
}