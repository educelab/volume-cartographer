//
// Created by Hannah Hatch on 8/15/16.
//

#include <iostream>
#include "common/vc_defines.h"
#include "meshing/QuadricEdgeCollapseResampling.h"
#include "common/io/objWriter.h"
#include "common/shapes/Plane.h"

int main(int argc, char*argv[])
{
    volcart::io::objWriter writer;

    volcart::shapes::Plane plane(10,10);

    volcart::meshing::QuadricEdgeCollapseResampling Resampler;
    Resampler.setMesh(plane.itkMesh());
    Resampler.compute();

    writer.setPath("QuadricEdgeCollapse_Plane.obj");
    writer.setMesh(Resampler.getMesh());
    writer.write();

    return EXIT_SUCCESS;
}