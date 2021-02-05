#include <boost/filesystem.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;
namespace fs = boost::filesystem;

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
    std::cout << "Reading mesh..." << std::endl;
    vc::io::OBJReader reader;
    reader.setPath(meshPath);
    auto mesh = reader.read();

    // ABF
    std::cout << "Computing parameterization..." << std::endl;
    vc::texturing::AngleBasedFlattening abf;
    abf.setMesh(mesh);
    abf.compute();

    // Get UV map
    auto uvMap = abf.getUVMap();
    auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
    auto height = static_cast<size_t>(std::ceil(width / uvMap.ratio().aspect));

    // PPM
    vc::texturing::PPMGenerator p;
    p.setDimensions(height, width);
    p.setMesh(mesh);
    p.setUVMap(uvMap);
    auto label = "Generating PPM (" + std::to_string(width) + "x" +
                 std::to_string(height) + "):";
    vc::ReportProgress(p, label);
    p.compute();

    // Write PPM
    std::cout << "Writing per-pixel map..." << std::endl;
    vc::PerPixelMap::WritePPM(ppmPath, p.getPPM());

    return EXIT_SUCCESS;
}
