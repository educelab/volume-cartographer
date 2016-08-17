//
// Created by Hannah Hatch on 8/15/16.
//

#include <iostream>
#include "common/vc_defines.h"
#include "meshing/QuadricEdgeCollapseResampling.h"
#include "common/io/ply2itk.h"
#include "common/io/objWriter.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Arch.h"
#include "common/shapes/Cone.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Sphere.h"
#include "meshing/CalculateNormals.h"

int main(int argc, char*argv[])
{
    volcart::io::objWriter writer;

    //Imports mesh from file
    volcart::shapes::Plane plane(10,10);

    volcart::meshing::QuadricEdgeCollapseResampling ResamplerP;
    ResamplerP.setMesh(plane.itkMesh());
    ResamplerP.compute(plane.itkMesh()->GetNumberOfPoints() /2);

    VC_MeshType::Pointer Resample = ResamplerP.getMesh();
    volcart::meshing::CalculateNormals calcNorm( Resample );
    calcNorm.compute();
    Resample= calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Plane.obj");
    writer.setMesh(Resample);
    writer.write();

    volcart::meshing::QuadricEdgeCollapseResampling ResamplerA;
    volcart::shapes::Arch arch(100,100);

    ResamplerA.setMesh(arch.itkMesh());
    ResamplerA.compute(arch.itkMesh()->GetNumberOfPoints() /2);

    Resample = ResamplerA.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample= calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Arch.obj");
    writer.setMesh(Resample);
    writer.write();

    volcart::meshing::QuadricEdgeCollapseResampling ResamplerC;
    volcart::shapes::Cone cone(1000,1000);

    ResamplerC.setMesh(cone.itkMesh());
    ResamplerC.compute(cone.itkMesh()->GetNumberOfPoints() /2);

    Resample = ResamplerC.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample= calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Cone.obj");
    writer.setMesh(Resample);
    writer.write();

    volcart::meshing::QuadricEdgeCollapseResampling ResamplerB;
    volcart::shapes::Cube cube;

    ResamplerB.setMesh(cube.itkMesh());
    ResamplerB.compute(cube.itkMesh()->GetNumberOfPoints() /2);

    Resample = ResamplerB.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample= calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Cube.obj");
    writer.setMesh(Resample);
    writer.write();

    volcart::meshing::QuadricEdgeCollapseResampling ResamplerS;
    volcart::shapes::Sphere sphere(30,3);

    ResamplerS.setMesh(sphere.itkMesh());
    ResamplerS.compute(sphere.itkMesh()->GetNumberOfPoints() /2);

    Resample = ResamplerS.getMesh();
    calcNorm.setMesh(Resample);
    calcNorm.compute();
    Resample= calcNorm.getMesh();
    writer.setPath("QuadricEdgeCollapse_Sphere.obj");
    writer.setMesh(Resample);
    writer.write();

    return EXIT_SUCCESS;
}