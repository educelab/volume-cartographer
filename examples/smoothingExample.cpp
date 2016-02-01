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

#include "smoothNormals.h"
#include "vc_defines.h"
#include "shapes.h"
#include "itkMeshFileWriter.h"

int main(){

    //smoothing factor
    double factor = 4;

    //Declare VC_MeshType::Pointer objects to hold the various shapes
    VC_MeshType::Pointer inputMesh, outputMesh;

    //Declare shape objects
    volcart::shapes::Plane plane;
    volcart::shapes::Cube cube;
    volcart::shapes::Arch arch;
    volcart::shapes::Sphere sphere;

    //Initialize itk writer
    itk::MeshFileWriter<VC_MeshType>::Pointer itkwriter = itk::MeshFileWriter<VC_MeshType>::New();

    ///////////
    // Plane //
    ///////////

    inputMesh = plane.itkMesh();

    //call smoothNormals() and save output to file
    outputMesh = volcart::meshing::smoothNormals(inputMesh, factor);

    //write outputMesh to file

    itkwriter->SetInput(outputMesh);
    itkwriter->SetFileTypeAsASCII();
    itkwriter->SetFileName("smoothedPlane.obj");
    itkwriter->Write();

    ///////////
    // Cube  //
    ///////////

    inputMesh = cube.itkMesh();

    //call smoothNormals() and save output to file
    outputMesh = volcart::meshing::smoothNormals(inputMesh, factor);

    //write outputMesh to file

    itkwriter->SetInput(outputMesh);
    itkwriter->SetFileTypeAsASCII();
    itkwriter->SetFileName("smoothedCube.obj");
    itkwriter->Write();

    ///////////
    // Arch  //
    ///////////

    inputMesh = arch.itkMesh();

    //call smoothNormals() and save output to file
    outputMesh = volcart::meshing::smoothNormals(inputMesh, factor);

    //write outputMesh to file

    itkwriter->SetInput(outputMesh);
    itkwriter->SetFileTypeAsASCII();
    itkwriter->SetFileName("smoothedArch.obj");
    itkwriter->Write();

    ////////////
    // Sphere //
    ////////////

    inputMesh = sphere.itkMesh();

    //call smoothNormals() and save output to file
    outputMesh = volcart::meshing::smoothNormals(inputMesh, factor);

    //write outputMesh to file

    itkwriter->SetInput(outputMesh);
    itkwriter->SetFileTypeAsASCII();
    itkwriter->SetFileName("smoothedSphere.obj");
    itkwriter->Write();

    return 0;
}