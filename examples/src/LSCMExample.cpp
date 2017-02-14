//
// Created by Seth Parker on 6/28/15.
//
/*
 * Purpose: Run volcart::texturing::LeastSquaresConformalMapping() and write
 * results to file for each shape.
 *          Saved file wills be read in by the lscmTest.cpp file under
 * v-c/testing/texturing.
 */

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"
#include "texturing/LeastSquaresConformalMapping.hpp"

int main()
{
    // Create the mesh writer
    volcart::io::OBJWriter mesh_writer;

    // Setup the test objects
    volcart::shapes::Plane plane;
    volcart::shapes::Arch arch;
    volcart::texturing::LeastSquaresConformalMapping lscm;

    //// Plane tests ////
    // Plane ABF & LSCM
    lscm.setMesh(plane.itkMesh());
    lscm.compute();
    mesh_writer.setPath("lscm_Plane.obj");
    mesh_writer.setMesh(lscm.getMesh());
    mesh_writer.write();

    //// Arch tests ////
    // Arch ABF & LSCM
    lscm.setMesh(arch.itkMesh());
    lscm.compute();
    mesh_writer.setPath("lscm_Arch.obj");
    mesh_writer.setMesh(lscm.getMesh());
    mesh_writer.write();

    return EXIT_SUCCESS;
}
