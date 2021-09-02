#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/types/VolumePkg.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

int main(int argc, char** argv)
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("input-mesh,i", po::value<std::string>()->required(),
           "Input mesh file")
        ("output-mesh,o", po::value<std::string>()->required(),
           "Output mesh file")
        ("axis,a", po::value<int>()->default_value(0),
           "Axis along which to flip:\n"
             "  0 = Vertical\n"
             "  1 = Horizontal\n"
             "  2 = Both")
        ("flip-image", "Flip the associated texture image");

    po::options_description all("Usage");
    all.add(required);
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
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get flipping direction
    auto index = parsed["axis"].as<int>();
    vc::UVMap::FlipAxis axis;
    int flipCode;
    switch (index) {
        case 0:
            axis = vc::UVMap::FlipAxis::Vertical;
            flipCode = index;
            break;
        case 1:
            axis = vc::UVMap::FlipAxis::Horizontal;
            flipCode = index;
            break;
        case 2:
            axis = vc::UVMap::FlipAxis::Both;
            flipCode = -1;
            break;
        default:
            std::cerr << "ERROR: Unknown flipping direction: " << index << "\n";
            return EXIT_FAILURE;
    }

    // Load mesh
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    vc::io::OBJReader reader;
    reader.setPath(inputPath);
    auto mesh = reader.read();

    // Update the UV map
    auto uvMap = reader.getUVMap();
    vc::UVMap::Flip(*uvMap, axis);

    // Load the texture image
    cv::Mat texture;
    try {
        texture = reader.getTextureMat();
    } catch (...) {
        std::cerr << "ERROR: Could not load texture image.\n" << std::endl;
        return EXIT_FAILURE;
    }

    if (parsed.count("flip-image") > 0 && !texture.empty()) {
        cv::flip(texture, texture, flipCode);
    }

    // Write the new mesh
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::io::OBJWriter writer;
    writer.setPath(outputPath);
    writer.setMesh(mesh);
    writer.setUVMap(uvMap);
    writer.setTexture(texture);
    writer.write();

    return EXIT_SUCCESS;
}
