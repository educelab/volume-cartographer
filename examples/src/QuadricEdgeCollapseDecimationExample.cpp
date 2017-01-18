//
// Created by Hannah Hatch on 8/15/16.
//
#ifdef VC_USE_VCGLIB

#include <iostream>
#include "core/io/OBJWriter.hpp"
#include "core/shapes/Arch.hpp"
#include "core/shapes/Cone.hpp"
#include "core/shapes/Cube.hpp"
#include "core/shapes/Plane.hpp"
#include "core/shapes/Sphere.hpp"
#include "core/vc_defines.hpp"
#include "meshing/CalculateNormals.hpp"
#include "meshing/QuadricEdgeCollapseDecimation.hpp"

int main()
{
    volcart::meshing::QuadricEdgeCollapseDecimation Resampler;
    volcart::io::OBJWriter writer;

    // Plane
    volcart::shapes::Plane plane(10, 10);
    Resampler.setMesh(plane.itkMesh());
    Resampler.compute(plane.itkMesh()->GetNumberOfCells() / 2);

    auto Resample = Resampler.getMesh();
    volcart::meshing::CalculateNormals calcNorm(Resample);
    calcNorm.compute();
    Resample = calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Plane.obj");
    writer.setMesh(Resample);
    writer.write();

    // Arch
    volcart::shapes::Arch arch(100, 100);
    Resampler.setMesh(arch.itkMesh());
    Resampler.compute(arch.itkMesh()->GetNumberOfCells() / 2);

    Resample = Resampler.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample = calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Arch.obj");
    writer.setMesh(Resample);
    writer.write();

    // Cone
    volcart::shapes::Cone cone(1000, 1000);
    Resampler.setMesh(cone.itkMesh());
    Resampler.compute(cone.itkMesh()->GetNumberOfCells() / 2);

    Resample = Resampler.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample = calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Cone.obj");
    writer.setMesh(Resample);
    writer.write();

    // Cube
    volcart::shapes::Cube cube;
    Resampler.setMesh(cube.itkMesh());
    Resampler.compute(cube.itkMesh()->GetNumberOfCells() / 2);

    Resample = Resampler.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample = calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Cube.obj");
    writer.setMesh(Resample);
    writer.write();

    // Sphere
    volcart::shapes::Sphere sphere(30, 3);
    Resampler.setMesh(sphere.itkMesh());
    Resampler.compute(sphere.itkMesh()->GetNumberOfCells() / 2);

    Resample = Resampler.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample = calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Sphere.obj");
    writer.setMesh(Resample);
    writer.write();

    return EXIT_SUCCESS;
}
#endif
