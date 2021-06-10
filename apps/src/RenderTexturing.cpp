#include "vc/apps/render/RenderTexturing.hpp"

#include <exception>
#include <memory>
#include <sstream>

#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/Color.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/types/VolumetricMask.hpp"
#include "vc/core/util/ColorMaps.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/UVMapToITKMesh.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/FlatteningError.hpp"
#include "vc/texturing/IntegralTexture.hpp"
#include "vc/texturing/IntersectionTexture.hpp"
#include "vc/texturing/OrthographicProjectionFlattening.hpp"
#include "vc/texturing/PPMGenerator.hpp"
#include "vc/texturing/ScaleMarkerGenerator.hpp"
#include "vc/texturing/ThicknessTexture.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcm = volcart::meshing;
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
        ("uv-plot", po::value<std::string>(), "Plot the UV points and save "
            "it to the provided image path.")
        ("uv-plot-error", po::value<std::string>(), "Plot the UV L-stretch "
            "error metrics and save them to the provided image path. The "
            "provided filename will have \'_l2\' and \'lInf\' appended for "
            "each metric: e.g. providing \'foo.png\' to this argument will "
            "produce image files \'foo_l2.png\' and \'foo_lInf.png\'")
        ("uv-plot-error-legend", po::value<bool>()->default_value(true),
            "If enabled (default), add a legend to the UV error plot images");
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
                 "  2 = Integral\n"
                 "  3 = Thickness")
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

po::options_description GetThicknessOpts()
{
    // clang-format off
    po::options_description opts("Thickness Texture Options");
    opts.add_options()
        ("volume-mask", po::value<std::string>(),
            "Path to volumetric mask point set")
        ("normalize-output", po::value<bool>()->default_value(true),
            "Normalize the output image between [0, 1]. If enabled "
            "(default), the output file should be a TIFF file and the "
            "--tiff-floating-point flag should be provided.");
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
    vc::ITKMesh::Pointer uvMesh;
    if (parsed_.count("reuse-uv")) {
        if (!resampled) {
            uvMap = parsedUVMap_;
        } else {
            vc::Logger()->warn(
                "Provided '--reuse-uv' option, but input mesh has been "
                "resampled. Ignoring existing UV map.");
        }
    }

    // If we don't have a valid UV map yet, make one
    if (uvMap.empty()) {
        vc::Logger()->info("Computing parameterization");
        auto method =
            static_cast<FlatteningAlgorithm>(parsed_["uv-algorithm"].as<int>());

        // ABF and LSCM
        if (method == FlatteningAlgorithm::ABF ||
            method == FlatteningAlgorithm::LSCM) {
            vct::AngleBasedFlattening abf(mesh);
            abf.setUseABF(method == FlatteningAlgorithm::ABF);
            try {
                uvMesh = abf.compute();
            } catch (const std::exception& e) {
                vc::Logger()->critical(e.what());
                std::exit(EXIT_FAILURE);
            }
            uvMap = abf.getUVMap();
        }
        // Orthographic
        else if (method == FlatteningAlgorithm::Orthographic) {
            vct::OrthographicProjectionFlattening ortho;
            ortho.setMesh(mesh);
            uvMesh = ortho.compute();
            uvMap = ortho.getUVMap();
        }
    }

    // Rotate
    if (parsed_.count("uv-rotate") > 0) {
        auto theta = parsed_["uv-rotate"].as<double>();
        vc::Logger()->info("Rotating UV map {} degrees", theta);
        theta *= DEG_TO_RAD;
        vc::UVMap::Rotate(uvMap, theta);
    }

    // Flip
    if (parsed_.count("uv-flip") > 0) {
        auto axis =
            static_cast<vc::UVMap::FlipAxis>(parsed_["uv-flip"].as<int>());
        vc::Logger()->info("Flipping UV map");
        vc::UVMap::Flip(uvMap, axis);
    }

    // Need a UV mesh if we're plotting the UV map
    if (not uvMesh and
        (parsed_.count("uv-plot") > 0 or parsed_.count("uv-plot-error") > 0)) {
        vcm::UVMapToITKMesh uv2mesh;
        uv2mesh.setMesh(mesh);
        uv2mesh.setUVMap(uvMap);
        uv2mesh.setScaleToUVDimensions(true);
        uvMesh = uv2mesh.compute();
    }

    // Plot the UV Map
    if (parsed_.count("uv-plot") > 0) {
        vc::Logger()->info("Saving UV map plot");
        fs::path uvPlotPath = parsed_["uv-plot"].as<std::string>();
        cv::imwrite(uvPlotPath.string(), vc::UVMap::Plot(uvMap, uvMesh));
    }

    // Plot the UV error maps
    if (parsed_.count("uv-plot-error") > 0) {
        auto errMetrics = vct::LStretch(mesh, uvMesh);
        // Invert metrics. See note in LStretchMetrics
        errMetrics = vct::InvertLStretchMetrics(errMetrics);
        vc::Logger()->info(
            "Calculated L-stretch flattening error :: Inverted Mesh L2: {:.5g} "
            "|| Inverted Mesh LInf: {:.5g}",
            errMetrics.l2, errMetrics.lInf);

        // Generate cell map since we don't have one yet
        vc::Logger()->info("Generating per-face UV error plots");
        auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
        auto height = static_cast<size_t>(std::ceil(uvMap.ratio().height));
        auto cellMap = vct::GenerateCellMap(uvMesh, uvMap, height, width);

        // Generate the error plots
        auto legend = parsed_["uv-plot-error-legend"].as<bool>();
        auto errPlots = vct::PlotLStretchError(
            errMetrics, cellMap, vc::ColorMap::Plasma, legend);

        // Save the plots
        vc::Logger()->info("Saving UV error plots");
        fs::path plotBase = parsed_["uv-plot-error"].as<std::string>();
        auto l2File =
            plotBase.stem().string() + "_l2" + plotBase.extension().string();
        auto l2Path = plotBase.parent_path() / l2File;
        cv::imwrite(l2Path.string(), errPlots[0]);

        auto lInfFile =
            plotBase.stem().string() + "_lInf" + plotBase.extension().string();
        auto lInfPath = plotBase.parent_path() / lInfFile;
        cv::imwrite(lInfPath.string(), errPlots[1]);
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

    ///// Generate the PPM /////
    auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<size_t>(std::ceil(uvMap.ratio().height));
    vct::PPMGenerator ppmGen;
    ppmGen.setMesh(mesh);
    ppmGen.setUVMap(uvMap);
    ppmGen.setDimensions(height, width);
    ppmGen.setShading(shading);

    // Progress tracker
    vc::Logger()->info("Rendering PPM");
    auto ppm = ppmGen.compute();

    // Save the PPM
    if (parsed_.count("output-ppm")) {
        vc::Logger()->info("Writing PPM");
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
    // Report selected generic options
    std::stringstream ss;
    ss << "Texturing neighborhood parameters :: ";
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

    // Set method specific parameters
    vct::TexturingAlgorithm::Pointer textureGeneric;
    if (method == Method::Intersection) {
        textureGeneric = vct::IntersectionTexture::New();
    }

    else if (method == Method::Composite) {
        auto textureSpecific = vct::CompositeTexture::New();
        textureSpecific->setFilter(filter);
        textureSpecific->setGenerator(generator);
        textureGeneric = textureSpecific;
    }

    else if (method == Method::Integral) {
        auto textureSpecific = vct::IntegralTexture::New();
        textureSpecific->setGenerator(generator);
        textureSpecific->setWeightMethod(weightType);
        textureSpecific->setLinearWeightDirection(weightDirection);
        textureSpecific->setExponentialDiffExponent(weightExponent);
        textureSpecific->setExponentialDiffBaseMethod(expoDiffBaseMethod);
        textureSpecific->setExponentialDiffBaseValue(expoDiffBase);
        textureSpecific->setClampValuesToMax(clampToMax);
        if (clampToMax) {
            textureSpecific->setClampMax(
                parsed_["clamp-to-max"].as<uint16_t>());
        }
        textureGeneric = textureSpecific;
    }

    else if (method == Method::Thickness) {
        // Load mask
        if (maskPath.empty()) {
            vc::Logger()->critical(
                "Selected Thickness texturing, but did not provide volume mask "
                "path.");
            std::exit(EXIT_FAILURE);
        }
        auto pts = vc::PointSetIO<cv::Vec3i>::ReadPointSet(maskPath);
        auto mask = vc::VolumetricMask::New(pts);

        auto textureSpecific = vct::ThicknessTexture::New();
        textureSpecific->setSamplingInterval(interval);
        textureSpecific->setNormalizeOutput(normalize);
        textureSpecific->setVolumetricMask(mask);
        textureGeneric = textureSpecific;
    }

    // Set method generic parameters
    textureGeneric->setVolume(volume_);
    textureGeneric->setPerPixelMap(ppm);

    // Setup progress tracker
    if (parsed_["progress"].as<bool>()) {
        vc::ReportProgress(*textureGeneric, "Texturing:");
        vc::Logger()->debug("Rendering texture image");
    } else {
        vc::Logger()->info("Rendering texture image");
    }

    // Execute texture algorithm
    auto texture = textureGeneric->compute();

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
            vc::Logger()->warn(
                "Specified reference image scale marker but missing required "
                "'--scale-marker-ref-*' options. Falling back to default scale "
                "marker.");
            scaleGen.setScaleType(ScaleGenerator::Type::Metric);
        }

        // Generate scale marker
        vc::Logger()->info("Generating scale markers");
        cv::Mat result;
        try {
            result = scaleGen.compute();
        } catch (const std::exception& e) {
            vc::Logger()->error(e.what());
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