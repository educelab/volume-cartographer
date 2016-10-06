//
// Created by Ryan Taber on 2/17/16.
//

/*
 * Purpose: Run volcart::objWriter() to write Plane itk mesh to file
 *          Saved file will be read in by the objWriterTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "common/io/objWriter.h"
#include "common/shapes/Plane.h"
#include "common/vc_defines.h"

int main(int argc, char* argv[])
{

    // init shapes
    volcart::shapes::Plane Plane;
    volcart::ITKMesh::Pointer in_PlaneITKMesh = Plane.itkMesh();

    // declare writer
    volcart::io::objWriter mesh_writer;

    // write mesh to file
    mesh_writer.setPath("OBJWriterPlaneData.obj");
    mesh_writer.setMesh(in_PlaneITKMesh);
    mesh_writer.write();

    return EXIT_SUCCESS;
}
