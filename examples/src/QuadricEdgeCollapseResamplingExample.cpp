//
// Created by Hannah Hatch on 8/15/16.
//

#include <iostream>
#include "common/vc_defines.h"
#include "meshing/QuadricEdgeCollapseResampling.h"
#include "common/io/ply2itk.h"
#include "common/io/objWriter.h"
#include "common/shapes/Plane.h"

int main(int argc, char*argv[])
{
    volcart::io::objWriter writer;

    //Imports mesh from file
    VC_MeshType::Pointer mesh = VC_MeshType::New();
    int width;
    int height;

    bool open_test = volcart::io::ply2itkmesh(argv[1], mesh, width, height);

    if (!open_test) {
        cerr << "Error Opening Input file" << endl;
        exit(EXIT_FAILURE);
    }

    volcart::meshing::QuadricEdgeCollapseResampling Resampler;
    Resampler.setMesh(mesh);
    Resampler.compute(3774);

    writer.setPath("QuadricEdgeCollapse_Plane.obj");
    writer.setMesh(Resampler.getMesh());
    writer.write();

    return EXIT_SUCCESS;
}