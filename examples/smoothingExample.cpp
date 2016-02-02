//
// Created by Ryan Taber on 2/1/16.
//

/*
 * Purpose: Run volcart::meshing::smoothNormals() and write results to file.
 *          Saved file will be read in by the smoothNormalsTest.cpp file under
 *          v-c/testing/meshing.
 *
 * Note: Uses a smoothing factor of 4, which is mirrored in the corresponding test.
 *       Creates an obj file for each of the derived shapes after calling smoothNormals().
 */

#include <io/objWriter.h>
#include "smoothNormals.h"
#include "vc_defines.h"
#include "shapes.h"


int main(){

    //smoothing factor
    double factor = 4;

    //Declare shape objects
    volcart::shapes::Plane plane;
    volcart::shapes::Cube cube;
    volcart::shapes::Arch arch;
    volcart::shapes::Sphere sphere;

    //Declare our obj writer
    volcart::io::objWriter mesh_writer;

    ///////////
    // Plane //
    ///////////

    mesh_writer.setPath( "plane.obj");
    mesh_writer.setMesh( plane.itkMesh() );
    mesh_writer.write();

    mesh_writer.setPath( "smoothedPlane.obj" );
    mesh_writer.setMesh( volcart::meshing::smoothNormals( plane.itkMesh(), factor ) );
    mesh_writer.write();

    ///////////
    // Cube  //
    ///////////

    //using our writer;
    mesh_writer.setPath( "smoothedCube.obj" );
    mesh_writer.setMesh( volcart::meshing::smoothNormals(cube.itkMesh(), factor) );
    mesh_writer.write();

    ///////////
    // Arch  //
    ///////////

    mesh_writer.setPath( "arch.obj");
    mesh_writer.setMesh( arch.itkMesh() );
    mesh_writer.write();

    mesh_writer.setPath("smoothedArch.obj");
    mesh_writer.setMesh( volcart::meshing::smoothNormals( arch.itkMesh(), factor ) );
    mesh_writer.write();

    ////////////
    // Sphere //
    ////////////

    mesh_writer.setPath("smoothedSphere.obj");
    mesh_writer.setMesh( volcart::meshing::smoothNormals( sphere.itkMesh(), factor) );
    mesh_writer.write();

    return 0;
}