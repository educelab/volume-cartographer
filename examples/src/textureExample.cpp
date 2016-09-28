//
// Created by Seth Parker on 6/24/15.
//

#include "common/io/objWriter.h"
#include "common/io/ply2itk.h"
#include "common/io/plyWriter.h"
#include "common/types/Texture.h"
#include "common/vc_defines.h"
#include "volumepkg/volumepkg.h"

#include "texturing/compositeTextureV2.h"
#include "texturing/simpleUV.h"

int main(int argc, char* argv[])
{
    VolumePkg vpkg(argv[1]);
    vpkg.setActiveSegmentation(argv[2]);

    // declare pointer to new Mesh object
    volcart::MeshType::Pointer inputMesh = volcart::MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if (!volcart::io::ply2itkmesh(
            vpkg.getMeshPath(), inputMesh, meshWidth, meshHeight)) {
        exit(EXIT_SUCCESS);
    };

    int width = 608 * 2;
    int height = 370 * 2;

    volcart::UVMap uvMap;
    uvMap.set(0, cv::Vec2d(0, 0));
    uvMap.set(1, cv::Vec2d(1, 0));
    uvMap.set(2, cv::Vec2d(0, 1));
    uvMap.set(3, cv::Vec2d(1, 1));
    uvMap.ratio(width, height);

    volcart::texturing::compositeTextureV2 compText(
        inputMesh, vpkg, uvMap, 1, width, height,
        volcart::CompositeOption::Minimum);

    volcart::io::objWriter mesh_writer;
    mesh_writer.setPath("compV2Test.obj");
    mesh_writer.setMesh(inputMesh);
    mesh_writer.setTexture(compText.texture().getImage(0));
    mesh_writer.setUVMap(compText.texture().uvMap());
    mesh_writer.write();

    return EXIT_SUCCESS;
}
