#include <cmath>
#include <limits>
#include <map>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/external/GetMemorySize.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vc = volcart;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 6;
static const double MAX_16BPC = std::numeric_limits<uint16_t>::max();

fs::path g_outputDir;
size_t g_numSliceChars;

void WriteBumpedSlice(const cv::Mat& slice, int index);

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("volume", po::value<std::string>()->required(), "Volume")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("bump-mask,m", po::value<std::string>()->required(), "Bump mask")
        ("output-dir,o", po::value<std::string>()->required(),"Output directory")
        ("cache-memory-limit", po::value<std::string>(), "Maximum size of the "
            "slice cache in bytes. Accepts the suffixes: (K|M|G|T)(B). "
            "Default: 50% of the total system memory.");


    po::options_description visOptions("Visualization Options");
    visOptions.add_options()
        ("bump,b", po::value<double>()->default_value(10), "Value to bump each pixel as percent of dynamic range");

    po::options_description all("Usage");
    all.add(required).add(visOptions);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 11) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::logger->error(e.what());
        return EXIT_FAILURE;
    }

    ///// Load the volume package /////
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    auto vpkg = vc::VolumePkg::New(volpkgPath);
    if (vpkg->version() != VOLPKG_SUPPORTED_VERSION) {
        vc::logger->error(
            "Volume Package is version {} but this program requires version "
            "{}. ",
            vpkg->version(), VOLPKG_SUPPORTED_VERSION);
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    try {
        volume = vpkg->volume(parsed["volume"].as<std::string>());
    } catch (const std::exception& e) {
        vc::logger->error(
            "Cannot load volume. Please check that the Volume Package has "
            "volumes and that the volume ID is correct.");
        vc::logger->error(e.what());
        return EXIT_FAILURE;
    }
    g_numSliceChars = std::to_string(volume->numSlices()).size();

    // Set the cache size
    size_t cacheBytes;
    if (parsed.count("cache-memory-limit")) {
        auto cacheSizeOpt = parsed["cache-memory-limit"].as<std::string>();
        cacheBytes = vc::MemorySizeStringParser(cacheSizeOpt);
    } else {
        cacheBytes = SystemMemorySize() / 2;
    }
    volume->setCacheMemoryInBytes(cacheBytes);
    vc::logger->info(
        "Volume Cache :: Capacity: {} || Size: {}", volume->getCacheCapacity(),
        vc::BytesToMemorySizeString(cacheBytes));

    ///// Load the output directory /////
    g_outputDir = parsed["output-dir"].as<std::string>();

    ///// Load the PPM and the bump mask /////
    vc::logger->info("Loading PPM: {}", parsed["ppm"].as<std::string>());
    auto ppm = vc::PerPixelMap::ReadPPM(parsed["ppm"].as<std::string>());
    vc::logger->info(
        "Loading bump mask: {}", parsed["bump-mask"].as<std::string>());
    auto bumpMask = cv::imread(parsed["bump-mask"].as<std::string>(), -1);

    // Calculate the bump offset
    auto bumpPerc = 100.0 / parsed["bump"].as<double>();
    auto bumpVal = (volume->max() - volume->min()) * bumpPerc;

    ///// Perform the bump /////
    // Get the mappings
    auto mappings = ppm.getMappings();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.pos[2] < rhs.pos[2];
        });

    // Iterate through the mappings
    int currentZ = -1;
    cv::Mat currentSlice;
    std::map<int, cv::Mat> bumpedSlices;
    for (const auto& pixel : mappings) {
        // Get integer coordinates
        auto x = static_cast<int>(std::floor(pixel.pos[0]));
        auto y = static_cast<int>(std::floor(pixel.pos[1]));
        auto z = static_cast<int>(std::floor(pixel.pos[2]));

        // Skip if this mapping is outside the volume bounds
        if (!volume->isInBounds(x, y, z)) {
            continue;
        }

        // Check if our slice index has changed
        if (z != currentZ) {
            if (!currentSlice.empty()) {
                // Add this to our image writing queue
                bumpedSlices[currentZ] = currentSlice;

                // Clear the queue if it's big enough
                if (bumpedSlices.size() == 100) {
                    for (const auto& s : bumpedSlices) {
                        WriteBumpedSlice(s.second, s.first);
                    }
                    bumpedSlices.clear();
                }
            }

            // Get the slice image for this PPM mapping
            currentZ = z;
            currentSlice = volume->getSliceDataCopy(currentZ);
        }

        // Use the bump mask as an opacity function on the bumpVal
        auto bumpOpacity = bumpMask.at<uint16_t>(pixel.y, pixel.x) / MAX_16BPC;
        auto bumped = currentSlice.at<uint16_t>(y, x) + bumpOpacity * bumpVal;
        currentSlice.at<uint16_t>(y, x) =
            static_cast<uint16_t>(std::min(bumped, MAX_16BPC));
    }

    // Write the last slice image
    WriteBumpedSlice(currentSlice, currentZ);
}

void WriteBumpedSlice(const cv::Mat& slice, int index)
{
    std::stringstream ss;
    ss << std::setw(g_numSliceChars) << std::setfill('0') << index << ".tif";
    fs::path p = g_outputDir / ss.str();
    vc::logger->info("Writing slice {}", p.string());
    vc::tiffio::WriteTIFF(p, slice);
}