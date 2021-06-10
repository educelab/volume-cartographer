#include <iostream>

#include <boost/program_options.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/ApplyLUT.hpp"
#include "vc/core/util/ColorMaps.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/String.hpp"

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
        ("output,o", po::value<std::string>(), "Output image file. Defaults to"
            "color map name")
        ("color-map,c", po::value<std::string>()->default_value("viridis"),
            "Color map options: magma, inferno, plasma, viridis, phase, "
            "bwr")
        ("invert", "If specified, invert the LUT mapping.")
        ("bins", po::value<std::size_t>()->default_value(256), "Number of "
            "bins in the color map LUT.")
        ("width,w", po::value<std::size_t>()->default_value(256),
            "Image width")
        ("height,h", po::value<std::size_t>()->default_value(36),
            "Image height");
    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 3) {
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

    // Load color map
    auto cmName = parsed["color-map"].as<std::string>();
    auto bins = parsed["bins"].as<std::size_t>();
    cv::Mat lut;
    try {
        lut = GetColorMapLUT(cmName, bins);
    } catch (const std::out_of_range& e) {
        Logger()->error("Unknown color map: {}", cmName);
        return EXIT_FAILURE;
    }

    // Apply color map
    bool invert = parsed.count("invert") > 0;
    auto height = parsed["height"].as<std::size_t>();
    auto width = parsed["width"].as<std::size_t>();
    auto bar = GenerateLUTScaleBar(lut, invert, height, width);

    // Write image
    fs::path outputPath;
    if (parsed.count("output") > 0) {
        outputPath = parsed["output"].as<std::string>();
    } else {
        outputPath = to_lower(cmName + ".png");
    }
    cv::imwrite(outputPath.string(), bar);
}