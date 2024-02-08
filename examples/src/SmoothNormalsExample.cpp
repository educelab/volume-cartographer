/*
 * Purpose: Run volcart::meshing::SmoothNormals() and write results to file.
 *          Saved file will be read in by the SmoothNormalsTest.cpp file under
 *          v-c/testing/meshing.
 *
 * Note: Uses a smoothing factor of 2, which is mirrored in the corresponding
 * test.
 *       Creates an obj file for each of the derived shapes after calling
 * SmoothNormals().
 */

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Cone.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Sphere.hpp"
#include "vc/meshing/SmoothNormals.hpp"

auto main() -> int
{

    // smoothing factor
    double factor = 2;

    // Declare shape objects
    volcart::shapes::Plane Plane;
    volcart::shapes::Cube Cube;
    volcart::shapes::Arch Arch;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    // Declare our obj writer
    volcart::io::OBJWriter mesh_writer;

    ///////////
    // Plane //
    ///////////

    mesh_writer.setPath("PlaneWithSmoothedNormals.obj");
    mesh_writer.setMesh(
        volcart::meshing::SmoothNormals(Plane.itkMesh(), factor));
    mesh_writer.write();

    //////////
    // Cube //
    //////////

    // using our writer;
    mesh_writer.setPath("CubeWithSmoothedNormals.obj");
    mesh_writer.setMesh(
        volcart::meshing::SmoothNormals(Cube.itkMesh(), factor));
    mesh_writer.write();

    //////////
    // Arch //
    //////////

    mesh_writer.setPath("ArchWithSmoothedNormals.obj");
    mesh_writer.setMesh(
        volcart::meshing::SmoothNormals(Arch.itkMesh(), factor));
    mesh_writer.write();

    ////////////
    // Sphere //
    ////////////

    mesh_writer.setPath("SphereWithSmoothedNormals.obj");
    mesh_writer.setMesh(
        volcart::meshing::SmoothNormals(Sphere.itkMesh(), factor));
    mesh_writer.write();

    //////////
    // Cone //
    //////////

    mesh_writer.setPath("ConeWithSmoothedNormals.obj");
    mesh_writer.setMesh(
        volcart::meshing::SmoothNormals(Cone.itkMesh(), factor));
    mesh_writer.write();

    return EXIT_SUCCESS;
}
