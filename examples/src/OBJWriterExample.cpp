//
// Created by Ryan Taber on 2/17/16.
//

/*
 * Purpose: Run volcart::OBJWriter() to write Plane itk mesh to file
 *          Saved file will be read in by the OBJWriterTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "core/io/OBJWriter.hpp"
#include "core/shapes/Plane.hpp"
#include "core/vc_defines.hpp"

int main()
{
    // init shapes
    volcart::shapes::Plane Plane;
    volcart::ITKMesh::Pointer in_PlaneITKMesh = Plane.itkMesh();

    // declare writer
    volcart::io::OBJWriter mesh_writer;

    // write mesh to file
    mesh_writer.setPath("OBJWriterPlaneData.obj");
    mesh_writer.setMesh(in_PlaneITKMesh);
    mesh_writer.write();

    return EXIT_SUCCESS;
}
