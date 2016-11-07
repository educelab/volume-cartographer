//
// Created by Seth Parker on 7/24/15.
//
/*
 * Purpose: Run volcart::texturing::abf() and write results to file for each
 * shape.
 *          Saved file will be read in by the abfTest.cpp file under
 * v-c/testing/texturing.
 */

#include "common/vc_defines.h"
#include "volumepkg/volumepkg.h"

#include "common/io/objWriter.h"
#include "common/shapes/Arch.h"
#include "common/shapes/Plane.h"
#include "texturing/AngleBasedFlattening.h"

int main()
{
    // Create the mesh writer
    volcart::io::objWriter mesh_writer;

    // Setup the test objects
    volcart::shapes::Plane plane;
    volcart::shapes::Arch arch;
    volcart::texturing::AngleBasedFlattening abf;

    //// Plane tests ////
    // Plane ABF & LSCM
    abf.setMesh(plane.itkMesh());
    abf.setUseABF(true);
    abf.compute();
    mesh_writer.setPath("abf_Plane.obj");
    mesh_writer.setMesh(abf.getMesh());
    mesh_writer.write();

    // Plane LSCM only
    abf.setMesh(plane.itkMesh());
    abf.setUseABF(false);
    abf.compute();
    mesh_writer.setPath("abf_Plane_LSCMOnly.obj");
    mesh_writer.setMesh(abf.getMesh());
    mesh_writer.write();

    //// Arch tests ////
    // Arch ABF & LSCM
    abf.setMesh(arch.itkMesh());
    abf.setUseABF(true);
    abf.compute();
    mesh_writer.setPath("abf_Arch.obj");
    mesh_writer.setMesh(abf.getMesh());
    mesh_writer.write();

    // Arch LSCM only
    abf.setMesh(arch.itkMesh());
    abf.setUseABF(false);
    abf.compute();
    mesh_writer.setPath("abf_Arch_LSCMOnly.obj");
    mesh_writer.setMesh(abf.getMesh());
    mesh_writer.write();

    return EXIT_SUCCESS;
}
