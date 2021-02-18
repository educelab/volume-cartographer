#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;
namespace fs = volcart::filesystem;

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0];
        std::cout << " mesh.obj output.ppm" << std::endl;
        return EXIT_FAILURE;
    }

    // Get inputs
    fs::path meshPath = argv[1];
    fs::path ppmPath = argv[2];

    // Load mesh
    vc::Logger()->info("Loading mesh");
    vc::io::OBJReader reader;
    reader.setPath(meshPath);
    auto mesh = reader.read();

    // ABF
    vc::texturing::AngleBasedFlattening abf;
    abf.setMesh(mesh);
    abf.compute();

    // Get UV map
    auto uvMap = abf.getUVMap();
    auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<size_t>(std::ceil(width / uvMap.ratio().aspect));

    // PPM
    vc::Logger()->info("Generating per-pixel map");
    vc::texturing::PPMGenerator p;
    p.setDimensions(height, width);
    p.setMesh(mesh);
    p.setUVMap(uvMap);
    p.compute();

    // Write PPM
    vc::Logger()->info("Writing per-pixel map");
    vc::PerPixelMap::WritePPM(ppmPath, p.getPPM());

    return EXIT_SUCCESS;
}
