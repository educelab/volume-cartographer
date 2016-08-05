//
// Created by Hannah Hatch on 7/25/16.
//

#include <iostream>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "OrderedResampling.h"
#include "io/objWriter.h"
#include "shapes.h"

int main(int argc, char* argv[]) {

    volcart::io::objWriter writer;

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
} //main