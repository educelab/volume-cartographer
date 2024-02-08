#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/ApplyLUT.hpp"
#include "vc/core/util/ColorMaps.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;
namespace po = boost::program_options;

namespace
{
auto ValueMax(int cvDepth) -> float
{
    switch (cvDepth) {
        case CV_8U:
            return std::numeric_limits<std::uint8_t>::max();
        case CV_16U:
            return std::numeric_limits<std::uint16_t>::max();
        case CV_32F:
            return 1.F;
        default:
            return 255;
    }
}
}  // namespace

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("input,i", po::value<std::string>()->required(), "Input image file")
        ("output,o", po::value<std::string>()->required(), "Output image file")
        ("color-map,c", po::value<std::string>()->default_value("viridis"),
            "Color map options: magma, inferno, plasma, viridis, phase, "
            "bwr")
        ("invert", "If specified, invert the LUT mapping.")
        ("bins", po::value<std::size_t>()->default_value(256),
            "Number of bins in the color map LUT.")
        ("mask", po::value<std::string>(),
            "Path to an image mask. The color mapper will only consider pixels "
            "contained in the mask when calculating automatic min/max values")
        ("min", po::value<float>(),
            "Minimum plotted value. Values less than this will be clamped to "
            "this value during color mapping. If --max is specified, this "
            "defaults to 0. If neither --min nor --max are specified, the "
            "range is automatically calculated from the input image.")
        ("max", po::value<float>(),
            "Maximum plotted value. Values greater than this will be clamped "
            "to this value during color mapping. If --min is specified, this "
            "defaults to the maximum value of the pixel type. If neither --min "
            "nor --max are specified, the range is automatically calculated "
            "from the input image.");
    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 5) {
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

    // Load image
    fs::path inputPath = parsed["input"].as<std::string>();
    Logger()->info("Loading image: {}", inputPath.string());
    cv::Mat input = cv::imread(inputPath.string(), cv::IMREAD_UNCHANGED);

    // Load color map
    auto cmName = parsed["color-map"].as<std::string>();
    auto bins = parsed["bins"].as<std::size_t>();
    cv::Mat lut;
    try {
        lut = GetColorMapLUT(cmName, bins);
    } catch (const std::out_of_range& e) {
        Logger()->error("Invalid color map: {}", cmName);
        return EXIT_FAILURE;
    }

    // Apply color map
    bool invert = parsed.count("invert") > 0;
    Logger()->info(
        "Applying color map: {} ({} bins, inverted: {})", cmName, bins, invert);
    cv::Mat map;
    auto haveMin = parsed.count("min") > 0;
    auto haveMax = parsed.count("max") > 0;
    if (haveMin or haveMax) {
        auto min = (haveMin) ? parsed["min"].as<float>() : 0;
        auto max =
            (haveMax) ? parsed["max"].as<float>() : ValueMax(input.depth());
        Logger()->info("Mapping range: [{:.5g}, {:.5g}]", min, max);
        map = ApplyLUT(input, lut, min, max, invert);
    } else {
        Logger()->info("Mapping range: Automatic");
        cv::Mat mask;
        if (parsed.count("mask") > 0) {
            mask = cv::imread(
                parsed["mask"].as<std::string>(), cv::IMREAD_GRAYSCALE);
        }
        map = ApplyLUT(input, lut, invert, mask);
    }

    // Write image
    fs::path outputPath = parsed["output"].as<std::string>();
    Logger()->info("Writing image: {}", outputPath.string());
    cv::imwrite(outputPath.string(), map);
}
