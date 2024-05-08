#include "vc/core/io/FileFilters.hpp"
#include "vc/core/io/MeshIO.hpp"
#include "vc/core/util/String.hpp"
#include "vc/meshing/ACVD.hpp"

namespace vc = volcart;

using Mode = vc::meshing::ACVD::Mode;

auto main(int argc, char* argv[]) -> int
{
    // Not enough opts
    if (argc < 4) {
        std::cerr << argv[0];
        std::cerr << " [input.ply | input.obj] ";
        std::cerr << "[output.ply | output.obj] ";
        std::cerr << "[num. vertices] ";
        std::cerr << "{enable anisotropic: [true | false]}";
        std::cerr << "\n";
        return EXIT_FAILURE;
    }

    // Get cmd line params
    const std::string inPath = argv[1];
    const std::string outPath = argv[2];
    auto numFaces = std::stoi(argv[3]);
    Mode mode{Mode::Isotropic};
    if (argc >= 5 and vc::to_bool(argv[4])) {
        mode = Mode::Anisotropic;
    }

    /** Load mesh **/
    std::cout << "Loading mesh..." << std::endl;
    auto loaded = vc::ReadMesh(inPath);

    /** Process mesh **/
    std::cout << "Processing mesh..." << std::endl;
    vc::meshing::ACVD remesh;
    remesh.setInputMesh(loaded.mesh);
    remesh.setNumberOfClusters(numFaces);
    remesh.setMode(mode);
    auto outputMesh = remesh.compute();

    /** Save mesh **/
    std::cout << "Saving mesh..." << std::endl;
    vc::WriteMesh(outPath, outputMesh);

    return EXIT_SUCCESS;
}
