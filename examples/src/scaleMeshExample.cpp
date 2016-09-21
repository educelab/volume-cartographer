//
// Created by Ryan Taber on 2/18/16.
//

/*
 * Purpose: Run volcart::meshing::scaleMesh() and write results to file
 *          for each shape.
 *          Saved file wills be read in by the scaleMeshTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "common/vc_defines.h"
#include "meshing/scaleMesh.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Arch.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Sphere.h"
#include "common/shapes/Cone.h"
#include <itkMeshFileWriter.h>
#include <common/io/objWriter.h>

int main(){

    //init shapes
    volcart::shapes::Plane Plane;
    volcart::shapes::Cube Cube;
    volcart::shapes::Arch Arch;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    //create itkMesh
    VC_MeshType::Pointer in_PlaneITKMesh = Plane.itkMesh();
    VC_MeshType::Pointer in_CubeITKMesh = Cube.itkMesh();
    VC_MeshType::Pointer in_ArchITKMesh = Arch.itkMesh();
    VC_MeshType::Pointer in_SphereITKMesh = Sphere.itkMesh();
    VC_MeshType::Pointer in_ConeITKMesh = Cone.itkMesh();

    //will hold results of scaleMesh
    VC_MeshType::Pointer out_PlaneMesh = VC_MeshType::New();
    VC_MeshType::Pointer out_CubeMesh = VC_MeshType::New();
    VC_MeshType::Pointer out_ArchMesh = VC_MeshType::New();
    VC_MeshType::Pointer out_SphereMesh = VC_MeshType::New();
    VC_MeshType::Pointer out_ConeMesh = VC_MeshType::New();

    //call scaleMesh() using scale Factor of 3 to match test cases
    volcart::meshing::scaleMesh(in_PlaneITKMesh, out_PlaneMesh, 3);
    volcart::meshing::scaleMesh(in_CubeITKMesh, out_CubeMesh, 3);
    volcart::meshing::scaleMesh(in_ArchITKMesh, out_ArchMesh, 3);
    volcart::meshing::scaleMesh(in_SphereITKMesh, out_SphereMesh, 3);
    volcart::meshing::scaleMesh(in_ConeITKMesh, out_ConeMesh, 3);

    //write scaled mesh results to file
    volcart::io::objWriter writer;

    //cycle through the shapes and write
    int ShapeCounter = 0;
    while (ShapeCounter < 5){

        if (ShapeCounter == 0){

            //write itk meshes
            writer.setMesh(out_PlaneMesh);
            writer.setPath("ScaledPlaneMesh.obj");
            writer.write();
        }
        else if (ShapeCounter == 1){

            writer.setMesh(out_CubeMesh);
            writer.setPath("ScaledCubeMesh.obj");
            writer.write();
        }
        else if (ShapeCounter == 2){

            writer.setMesh(out_ArchMesh);
            writer.setPath("ScaledArchMesh.obj");
            writer.write();
        }
        else if (ShapeCounter == 3){

            writer.setMesh(out_SphereMesh);
            writer.setPath("ScaledSphereMesh.obj");
            writer.write();
        }
        else if (ShapeCounter == 4){

            writer.setMesh(out_ConeMesh);
            writer.setPath("ScaledConeMesh.obj");
            writer.write();

            writer.setMesh(in_ConeITKMesh);
            writer.setPath("Cone.obj");
            writer.write();

        }

        ++ShapeCounter;
    }

    return EXIT_SUCCESS;
}


