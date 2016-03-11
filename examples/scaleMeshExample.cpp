//
// Created by Ryan Taber on 2/18/16.
//

/*
 * Purpose: Run volcart::meshing::scaleMesh() and write results to file
 *          for each shape.
 *          Saved file wills be read in by the scaleMeshTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "vc_defines.h"
#include "scaleMesh.h"
#include "shapes.h"
#include "itkMeshFileWriter.h"

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
    itk::MeshFileWriter<VC_MeshType>::Pointer itkwriter = itk::MeshFileWriter<VC_MeshType>::New();

    //cycle through the shapes and write
    int ShapeCounter = 0;
    while (ShapeCounter < 5){

        if (ShapeCounter == 0){

            //write itk meshes
            itkwriter->SetInput(out_PlaneMesh);
            itkwriter->SetFileTypeAsASCII();
            itkwriter->SetFileName("ScaledPlaneMesh.obj");
            itkwriter->Write();
        }
        else if (ShapeCounter == 1){

            itkwriter->SetInput(out_CubeMesh);
            itkwriter->SetFileTypeAsASCII();
            itkwriter->SetFileName("ScaledCubeMesh.obj");
            itkwriter->Write();
        }
        else if (ShapeCounter == 2){

            itkwriter->SetInput(out_ArchMesh);
            itkwriter->SetFileTypeAsASCII();
            itkwriter->SetFileName("ScaledArchMesh.obj");
            itkwriter->Write();
        }
        else if (ShapeCounter == 3){

            itkwriter->SetInput(out_SphereMesh);
            itkwriter->SetFileTypeAsASCII();
            itkwriter->SetFileName("ScaledSphereMesh.obj");
            itkwriter->Write();
        }
        else if (ShapeCounter == 4){

            itkwriter->SetInput(out_ConeMesh);
            itkwriter->SetFileTypeAsASCII();
            itkwriter->SetFileName("ScaledConeMesh.obj");
            itkwriter->Write();

            itkwriter->SetInput(in_ConeITKMesh);
            itkwriter->SetFileTypeAsASCII();
            itkwriter->SetFileName("Cone.obj");
            itkwriter->Write();
            
        }

        ++ShapeCounter;
    }

    return EXIT_SUCCESS;
}


