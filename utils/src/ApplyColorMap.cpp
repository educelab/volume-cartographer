#include <iostream>

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

int main(int argc, char* argv[])
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
        ("min", po::value<float>(),
            "Minimum plotted value. Values less than this will be clamped to "
            "this value during color mapping. Requires that --max is also "
            "specified. If not specified, the data min and max will be used.")
        ("max", po::value<float>(),
            "Maximum plotted value. Values greater than this will be clamped "
            "to this value during color mapping. Requires that --mins is also "
            "specified. If not specified, the data min and max will be used.");
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
    cv::Mat input = cv::imread(inputPath.string(), -1);

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
    if (parsed.count("min") > 0 and parsed.count("max") > 0) {
        auto min = parsed["min"].as<float>();
        auto max = parsed["max"].as<float>();
        Logger()->info("Mapping range: [{:.5g}, {:.5g}]", min, max);
        map = ApplyLUT(input, lut, min, max, invert);
    } else {
        Logger()->info("Mapping range: Automatic");
        map = ApplyLUT(input, lut, invert);
    }

    // Write image
    fs::path outputPath = parsed["output"].as<std::string>();
    Logger()->info("Writing image: {}", outputPath.string());
    cv::imwrite(outputPath.string(), map);
}