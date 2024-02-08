#include <iostream>

#include <boost/program_options.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vct = volcart::texturing;

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
            "Output mesh file")
        ("method,m", po::value<std::string>()->default_value("ABF"), "Flattening method: [ABF, LSCM]");

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

    bool useABF{true};
    auto method = parsed["method"].as<std::string>();
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);
    if (method == "lscm") {
        useABF = false;
    } else if (method != "abf") {
        std::cerr << "ERROR: Unknown flattening method: " << method;
        std::cerr << std::endl;
        return EXIT_FAILURE;
    }

    // Load mesh
    vc::Logger()->info("Loading mesh...");
    fs::path inputPath = parsed["input-mesh"].as<std::string>();
    vc::io::OBJReader reader;
    reader.setPath(inputPath);
    auto mesh = reader.read();
    vc::Logger()->info(
        "Mesh Loaded || Vertices: {} || Faces: {}", mesh->GetNumberOfPoints(),
        mesh->GetNumberOfCells());

    // Run ABF
    vct::AngleBasedFlattening abf;
    abf.setUseABF(useABF);
    abf.setMesh(mesh);
    mesh = abf.compute();

    vc::Logger()->info("Writing mesh...");
    fs::path outputPath = parsed["output-mesh"].as<std::string>();
    vc::io::OBJWriter writer;
    writer.setPath(outputPath);
    writer.setMesh(mesh);
    writer.write();
}