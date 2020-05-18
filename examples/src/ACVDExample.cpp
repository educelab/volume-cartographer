#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/OBJReader.hpp"
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/meshing/ACVD.hpp"

namespace vc = volcart;

int main(int argc, char* argv[])
{
    // Not enough opts
    if (argc != 4) {
        std::cerr << argv[0]
                  << " [input.ply | input.obj] [output.obj] [num. vertices]"
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Get cmd line params
    std::string inPath = argv[1];
    std::string outPath = argv[2];
    auto numFaces = std::stoi(argv[3]);

    /** Load mesh **/
    std::cout << "Loading mesh..." << std::endl;
    vc::ITKMesh::Pointer mesh;
    // OBJs
    if (vc::io::FileExtensionFilter(inPath, {"obj"})) {
        vc::io::OBJReader r;
        r.setPath(inPath);
        mesh = r.read();
    }

    // PLYs
    else if (vc::io::FileExtensionFilter(inPath, {"ply"})) {
        vc::io::PLYReader r(inPath);
        mesh = r.read();
    }

    // Can't load file
    else {
        std::cerr << "ERROR: Mesh file not of supported type: ";
        std::cerr << inPath << std::endl;
        std::exit(EXIT_FAILURE);
    }

    /** Process mesh **/
    std::cout << "Processing mesh..." << std::endl;
    vc::meshing::ACVD remesh;
    remesh.setInputMesh(mesh);
    remesh.setNumberOfClusters(numFaces);
    auto outputMesh = remesh.compute();

    /** Save mesh **/
    std::cout << "Saving mesh..." << std::endl;
    vc::io::OBJWriter writer(outPath, outputMesh);
    writer.write();

    return EXIT_SUCCESS;
}
