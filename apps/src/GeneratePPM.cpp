#include <boost/program_options.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;
namespace fs = volcart::filesystem;
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("input-mesh,i", po::value<std::string>()->required(),
            "Path to the input mesh")
        ("output-ppm,o", po::value<std::string>()->required(),
            "Path for the output ppm")
        ("uv-reuse", "If input-mesh is specified, attempt to use its existing "
            "UV map instead of generating a new one.");
    // clang-format on

    // parsed will hold the values of all parsed options as a Map
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
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Get inputs
    fs::path meshPath = parsed["input-mesh"].as<std::string>();
    fs::path ppmPath = parsed["output-ppm"].as<std::string>();

    // Load mesh
    vc::Logger()->info("Loading mesh");
    auto meshFile = vc::ReadMesh(meshPath);
    auto mesh = meshFile.mesh;
    auto uvMap = meshFile.uv;

    // Generate UV map
    auto genUV = parsed.count("uv-reuse") == 0;
    if (genUV or not uvMap) {
        // ABF
        vc::texturing::AngleBasedFlattening abf;
        abf.setMesh(mesh);
        abf.compute();

        // Get UV map
        uvMap = abf.getUVMap();
    }

    auto width = static_cast<size_t>(std::ceil(uvMap->ratio().width));
    auto height = static_cast<size_t>(std::ceil(width / uvMap->ratio().aspect));

    // PPM
    vc::Logger()->info("Generating per-pixel map");
    vc::texturing::PPMGenerator p;
    p.setDimensions(height, width);
    p.setMesh(mesh);
    p.setUVMap(uvMap);
    p.compute();

    // Write PPM
    vc::Logger()->info("Writing per-pixel map");
    vc::PerPixelMap::WritePPM(ppmPath, *p.getPPM());

    return EXIT_SUCCESS;
}
