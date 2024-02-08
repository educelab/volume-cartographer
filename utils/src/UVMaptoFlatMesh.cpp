#include <iostream>

#include <boost/program_options.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/UVMapToITKMesh.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

auto main(int argc, char** argv) -> int
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
         "Output mesh file");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
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

    // Get the parsed options
    // Load mesh
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    vc::io::OBJReader reader;
    reader.setPath(inputPath);
    auto mesh = reader.read();

    // get UVMap
    auto uvMap = reader.getUVMap();
    if (uvMap->empty()) {
        vc::Logger()->error("Input mesh has empty or null UV map.");
        return EXIT_FAILURE;
    }

    // Call UVMap to ITKMesh
    vc::meshing::UVMapToITKMesh mesher;
    mesher.setScaleToUVDimensions(true);
    mesher.setUVMap(uvMap);
    mesher.setMesh(mesh);
    vc::Logger()->info("Generating mesh from UV map...");
    auto itkMesh = mesher.compute();

    // Write the new mesh
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::io::OBJWriter writer;
    writer.setPath(outputPath);
    writer.setMesh(itkMesh);

    auto texture = reader.getTextureMat();
    if (!texture.empty()) {
        writer.setUVMap(uvMap);
        writer.setTexture(reader.getTextureMat());
    } else {
        vc::Logger()->warn("Input mesh has empty or null texture image.");
    }
    writer.write();

    return EXIT_SUCCESS;
}
