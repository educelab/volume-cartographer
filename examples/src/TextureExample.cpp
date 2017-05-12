//
// Created by Seth Parker on 6/24/15.
//

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"
#include "vc/texturing/CompositeTextureV2.hpp"

int main(int /*argc*/, char* argv[])
{

    volcart::VolumePkg vpkg(argv[1]);
    vpkg.setActiveSegmentation(argv[2]);

    // declare pointer to new Mesh object
    auto inputMesh = volcart::ITKMesh::New();

    // try to convert the ply to an ITK mesh
    volcart::io::PLYReader reader(vpkg.getMeshPath());
    try {
        reader.read();
        inputMesh = reader.getMesh();
    } catch (std::exception e) {
        std::cerr << e.what() << std::endl;
        exit(EXIT_SUCCESS);
    }

    int width = 608 * 2;
    int height = 370 * 2;

    volcart::UVMap uvMap;
    uvMap.set(0, cv::Vec2d(0, 0));
    uvMap.set(1, cv::Vec2d(1, 0));
    uvMap.set(2, cv::Vec2d(0, 1));
    uvMap.set(3, cv::Vec2d(1, 1));
    uvMap.ratio(width, height);

    volcart::texturing::CompositeTextureV2 compText(
        inputMesh, vpkg, uvMap, 1, width, height,
        volcart::CompositeOption::Minimum);

    volcart::io::OBJWriter mesh_writer;
    mesh_writer.setPath("compV2Test.obj");
    mesh_writer.setMesh(inputMesh);
    mesh_writer.setTexture(compText.getTexture().image(0));
    mesh_writer.setUVMap(compText.getTexture().uvMap());
    mesh_writer.write();

    return EXIT_SUCCESS;
}
