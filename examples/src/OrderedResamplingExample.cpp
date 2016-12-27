//
// Created by Hannah Hatch on 7/25/16.
//

#include <iostream>

#include "core/io/OBJWriter.h"
#include "core/shapes/Arch.h"
#include "core/shapes/Plane.h"
#include "core/vc_defines.h"
#include "meshing/OrderedResampling.h"

int main(int argc, char* argv[])
{

    volcart::io::OBJWriter writer;

    volcart::shapes::Plane plane(10, 10);
    int width = plane.orderedWidth();
    int height = plane.orderedHeight();

    volcart::meshing::OrderedResampling orderedResampling;
    orderedResampling.setMesh(plane.itkMesh(), width, height);
    orderedResampling.compute();

    writer.setPath("OrderedResampling_Plane.obj");
    writer.setMesh(orderedResampling.getOutputMesh());
    writer.write();

    volcart::shapes::Arch arch(20, 20);
    width = arch.orderedWidth();
    height = arch.orderedHeight();

    orderedResampling.setMesh(arch.itkMesh(), width, height);
    orderedResampling.compute();

    writer.setPath("OrderedResampling_Arch.obj");
    writer.setMesh(orderedResampling.getOutputMesh());
    writer.write();

    return EXIT_SUCCESS;
}  // main
