#include <sstream>

#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

#include "vc/app_support/GetMemorySize.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/ImageIO.hpp"
#include "vc/core/neighborhood/LineGenerator.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/texturing/LayerTexture.hpp"

namespace vc = volcart;
namespace fs = volcart::filesystem;
namespace po = boost::program_options;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;

using volcart::enumerate;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: The first volume in the "
            "volume package.")
        ("output-dir,o", po::value<std::string>()->required(),
            "Output directory for layer images.")
        ("output-ppm", po::value<std::string>(), "Create and save a new PPM "
            "that maps to the layer subvolume.");

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

    po::options_description performanceOptions("Performance Options");
    performanceOptions.add_options()
        ("cache-memory-limit", po::value<std::string>(), "Maximum size of the "
            "slice cache in bytes. Accepts the suffixes: (K|M|G|T)(B). "
            "Default: 50% of the total system memory.");

    po::options_description all("Usage");
    all.add(required)
        .add(filterOptions)
        .add(ppmOptions)
        .add(performanceOptions);
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
    fs::path inputPPMPath = parsed["ppm"].as<std::string>();

    // Check for output file
    auto outputPath = fs::canonical(parsed["output-dir"].as<std::string>());
    if (!fs::is_directory(outputPath) || !fs::exists(outputPath)) {
        std::cerr << "Provided output path is not a directory or does not exist"
                  << std::endl;
        return EXIT_FAILURE;
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
    double radius;
    if (parsed.count("radius")) {
        radius = parsed["radius"].as<double>();
    } else {
        radius = vpkg.materialThickness() / 2 / volume->voxelSize();
    }

    auto interval = parsed["interval"].as<double>();
    auto direction = static_cast<vc::Direction>(parsed["direction"].as<int>());

    // Read the ppm
    std::cout << "Loading PPM..." << std::endl;
    auto ppm = vc::PerPixelMap::New(vc::PerPixelMap::ReadPPM(inputPPMPath));

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

    std::cout << "Writing layers..." << std::endl;
    fs::path filepath;
    for (const auto [i, image] : enumerate(texture)) {
        filepath = outputPath / (std::to_string(i) + ".png");
        vc::WriteImage(filepath, image);
    }

    if (parsed.count("output-ppm") > 0) {
        std::cout << "Generating new PPM..." << std::endl;
        fs::path outputPPMPath = parsed["output-ppm"].as<std::string>();

        // Setup new PPM
        auto height = ppm->height();
        auto width = ppm->width();
        vc::PerPixelMap newPPM(height, width);
        newPPM.setMask(ppm->mask());

        // Fill new PPM
        auto z = static_cast<double>(texture.size() - 1) / 2.0;
        auto normal = (parsed.count("negative-normal") > 0) ? -1.0 : 1.0;
        for (size_t y = 0; y < height; y++) {
            for (size_t x = 0; x < width; x++) {
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
        }

        // Write the new PPM
        std::cout << "Writing new PPM..." << std::endl;
        vc::PerPixelMap::WritePPM(outputPPMPath, newPPM);
    }
}
