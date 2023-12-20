#include <cstddef>
#include <sstream>

#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

#include "vc/app_support/GeneralOptions.hpp"
#include "vc/app_support/GetMemorySize.hpp"
#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/ImageIO.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Transforms.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/core/util/String.hpp"
#include "vc/texturing/LayerTexture.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;
namespace po = boost::program_options;

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
    // All command line options
    // clang-format off
    po::options_description ioOpts("Input/Output Options");
    ioOpts.add_options()
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: The first volume in the "
            "volume package.")
        ("output-dir,o", po::value<std::string>()->required(),
            "Output directory for layer images.")
        ("output-ppm", po::value<std::string>(), "Create and save a new PPM "
            "that maps to the layer volume.")
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

    po::options_description ppmOptions("PPM Generation Options");
    ppmOptions.add_options()
        ("negative-normal", "Orient normals in the negative Z direction. By "
            "default, normals in the new PPM are oriented in the positive "
            "Z direction.");

    po::options_description all("Usage");
    all.add(GetGeneralOpts())
        .add(ioOpts)
        .add(::GetTransformOpts())
        .add(filterOptions)
        .add(ppmOptions);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
        std::cout << all << "\n";
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Get the parsed options
    const fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    const fs::path inputPPMPath = parsed["ppm"].as<std::string>();

    // Check for output file
    auto outDir = fs::weakly_canonical(parsed["output-dir"].as<std::string>());
    if (outDir.has_extension()) {
        Logger()->error(
            "Provided output path is not a directory: {}", outDir.string());
        return EXIT_FAILURE;
    }
    if (not fs::exists(outDir)) {
        auto success = fs::create_directory(outDir);
        if (not success) {
            Logger()->error(
                "Could not create output directory: {}. Check that parent "
                "exists and is writable.",
                outDir.string());
            return EXIT_FAILURE;
        }
    }
    auto imgFmt = to_lower_copy(parsed["image-format"].as<std::string>());
    WriteImageOpts writeOpts;
    if (parsed.count("compression") > 0) {
        writeOpts.compression = parsed["compression"].as<int>();
    } else {
        // Default for tiff in this app: No compression
        if (imgFmt == "tif" or imgFmt == "tiff") {
            writeOpts.compression = 1;
        }
    }

    ///// Load the volume package /////
    VolumePkg vpkg(volpkgPath);
    if (vpkg.version() < VOLPKG_MIN_VERSION) {
        Logger()->error(
            "Volume Package is version {} but this program requires version "
            "{}+. ",
            vpkg.version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    Volume::Pointer volume;
    try {
        if (parsed.count("volume") > 0) {
            volume = vpkg.volume(parsed["volume"].as<std::string>());
        } else {
            volume = vpkg.volume();
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
    double radius;
    if (parsed.count("radius") > 0) {
        radius = parsed["radius"].as<double>();
    } else {
        radius = vpkg.materialThickness() / 2 / volume->voxelSize();
    }

    auto interval = parsed["interval"].as<double>();
    auto direction = static_cast<Direction>(parsed["direction"].as<int>());

    // Read the ppm
    Logger()->info("Loading PPM...");
    auto ppm = PerPixelMap::New(PerPixelMap::ReadPPM(inputPPMPath));

    ///// Transform the PPM /////
    if (parsed.count("transform") > 0) {
        // load the transform
        auto tfmId = parsed.at("transform").as<std::string>();
        Transform3D::Pointer tfm;
        if (vpkg.hasTransform(tfmId)) {
            tfm = vpkg.transform(tfmId);
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

    // Setup line generator
    auto line = LineGenerator::New();
    line->setSamplingRadius(radius);
    line->setSamplingInterval(interval);
    line->setSamplingDirection(direction);

    // Layer texture
    texturing::LayerTexture s;
    s.setVolume(volume);
    s.setPerPixelMap(ppm);
    s.setGenerator(line);

    // Progress reporting
    auto enableProgress = parsed["progress"].as<bool>();
    ProgressConfig cfg;
    if (parsed.count("progress-interval") > 0) {
        cfg.interval =
            DurationFromString(parsed["progress-interval"].as<std::string>());
    }

    if (enableProgress) {
        ReportProgress(s, "Generating layers:", cfg);
        Logger()->debug("Generating layers...");
    } else {
        Logger()->info("Generating layers...");
    }

    auto texture = s.compute();

    // Write the image sequence
    const fs::path filepath = outDir / ("{}." + imgFmt);
    if (enableProgress) {
        Logger()->debug("Writing layers...");
        auto progIt = ProgressWrap(texture, "Writing layers:", cfg);
        WriteImageSequence(filepath, progIt, writeOpts);
    } else {
        Logger()->info("Writing layers...");
        WriteImageSequence(filepath, texture, writeOpts);
    }

    if (parsed.count("output-ppm") > 0) {
        Logger()->info("Generating new PPM...");
        const fs::path outputPPMPath = parsed["output-ppm"].as<std::string>();

        // Setup new PPM
        auto height = ppm->height();
        auto width = ppm->width();
        PerPixelMap newPPM(height, width);
        newPPM.setMask(ppm->mask());
        newPPM.setCellMap(ppm->cellMap());

        // Fill new PPM
        auto z = static_cast<double>(texture.size() - 1) / 2.0;
        auto normal = (parsed.count("negative-normal") > 0) ? -1.0 : 1.0;
        for (auto [y, x] : range2D(height, width)) {
            if (!newPPM.hasMapping(y, x)) {
                continue;
            }
            newPPM(y, x) = {static_cast<double>(x),
                            static_cast<double>(y),
                            z,
                            0.0,
                            0.0,
                            normal};
        }

        // Write the new PPM
        Logger()->info("Writing new PPM...");
        PerPixelMap::WritePPM(outputPPMPath, newPPM);
    }
    Logger()->info("Done.");
}
