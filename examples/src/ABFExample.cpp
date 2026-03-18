/*
 * Purpose: Run volcart::texturing::AngleBasedFlattening and write results to
 * file for each shape. The ABF+LSCM and LSCM-only output files are read by
 * ABFTest.cpp as reference meshes. The ABF+HLSCM and HLSCM-only cases
 * demonstrate the new HierarchicalLSCM API but do not produce reference files.
 */

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Spiral.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"

auto main() -> int
{
    // Create the mesh writer
    volcart::io::OBJWriter mesh_writer;

    // Set up the test objects
    volcart::shapes::Plane plane;
    volcart::shapes::Arch arch;
    volcart::shapes::Spiral spiral(20, 10);
    volcart::texturing::AngleBasedFlattening abf;

    // Disable HLSCM for all reference-file cases so output matches ABFTest.cpp
    abf.setUseHLSCM(false);

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

    //// Spiral tests ////
    // Spiral ABF & LSCM
    abf.setMesh(spiral.itkMesh());
    abf.setUseABF(true);
    abf.compute();
    mesh_writer.setPath("abf_Spiral.obj");
    mesh_writer.setMesh(abf.getMesh());
    mesh_writer.write();

    // Spiral LSCM only
    abf.setMesh(spiral.itkMesh());
    abf.setUseABF(false);
    abf.compute();
    mesh_writer.setPath("abf_Spiral_LSCMOnly.obj");
    mesh_writer.setMesh(abf.getMesh());
    mesh_writer.write();

    //// HierarchicalLSCM examples ////
    abf.setUseHLSCM(true);

    // Plane ABF + HierarchicalLSCM
    abf.setMesh(plane.itkMesh());
    abf.setUseABF(true);
    abf.compute();

    // Plane HierarchicalLSCM only
    abf.setMesh(plane.itkMesh());
    abf.setUseABF(false);
    abf.compute();

    // Arch ABF + HierarchicalLSCM
    abf.setMesh(arch.itkMesh());
    abf.setUseABF(true);
    abf.compute();

    // Arch HierarchicalLSCM only
    abf.setMesh(arch.itkMesh());
    abf.setUseABF(false);
    abf.compute();

    return EXIT_SUCCESS;
}
