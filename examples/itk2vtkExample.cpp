//
// Created by Media Team on 6/24/15.
//


/*
 * Purpose: Run volcart::meshing::itk2vtk() and write results to file.
 *          Run volcart::meshing::vtk2itk() and write results to file.
 *          Saved file wills be read in by the itk2vtkTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "vc_defines.h"
#include "itk2vtk.h"
#include "shapes.h"

#include "vtkPLYWriter.h"
#include "itkMeshFileWriter.h"

int main( int argc, char* argv[] ) {

  //init shapes --> used by both conversion directions
  volcart::shapes::Plane Plane;
  volcart::shapes::Cube Cube;
  volcart::shapes::Arch Arch;
  volcart::shapes::Sphere Sphere;
  volcart::shapes::Cone Cone;

  //
  // itk2vtk converter
  //

  //init vtkPolyData* objects to hold output of itk2vtk conversions
  vtkPolyData *out_VTKPlane = vtkPolyData::New();
  vtkPolyData *out_VTKCube = vtkPolyData::New();
  vtkPolyData *out_VTKArch = vtkPolyData::New();
  vtkPolyData *out_VTKSphere = vtkPolyData::New();
  vtkPolyData *out_VTKCone = vtkPolyData::New();

  //itk2vtk() calls
  volcart::meshing::itk2vtk(Plane.itkMesh(), out_VTKPlane);
  volcart::meshing::itk2vtk(Cube.itkMesh(), out_VTKCube);
  volcart::meshing::itk2vtk(Arch.itkMesh(), out_VTKArch);
  volcart::meshing::itk2vtk(Sphere.itkMesh(), out_VTKSphere);
  volcart::meshing::itk2vtk(Cone.itkMesh(), out_VTKCone);


  //
  //vtk2itk conversions
  //

  //init VC_MeshType::Pointer objects to hold the output of vtk2itk conversions
  VC_MeshType::Pointer out_ITKPlane = VC_MeshType::New();
  VC_MeshType::Pointer out_ITKCube = VC_MeshType::New();
  VC_MeshType::Pointer out_ITKArch = VC_MeshType::New();
  VC_MeshType::Pointer out_ITKSphere = VC_MeshType::New();
  VC_MeshType::Pointer out_ITKCone = VC_MeshType::New();

  //vtk2itk() calls
  volcart::meshing::vtk2itk(Plane.vtkMesh(), out_ITKPlane);
  volcart::meshing::vtk2itk(Cube.vtkMesh(), out_ITKCube);
  volcart::meshing::vtk2itk(Arch.vtkMesh(), out_ITKArch);
  volcart::meshing::vtk2itk(Sphere.vtkMesh(), out_ITKSphere);
  volcart::meshing::vtk2itk(Cone.vtkMesh(), out_ITKCone);


  //
  // write meshes to file
  //

  vtkPLYWriter *vtkwriter = vtkPLYWriter::New();
  itk::MeshFileWriter<VC_MeshType>::Pointer itkwriter = itk::MeshFileWriter<VC_MeshType>::New();

  //cycle through the shapes and write
  int ShapeCounter = 0;
  while (ShapeCounter < 5){

    if (ShapeCounter == 0){

      //write vtk meshes
      vtkwriter->SetInputData(out_VTKPlane);
      vtkwriter->SetFileTypeToASCII();
      vtkwriter->SetFileName("ITKPlaneMeshConvertedToVTK.ply");
      vtkwriter->Write();

      //write itk meshes
      itkwriter->SetInput(out_ITKPlane);
      itkwriter->SetFileTypeAsASCII();
      itkwriter->SetFileName("VTKPlaneMeshConvertedToITK.obj");
      itkwriter->Write();
    }
    else if (ShapeCounter == 1){

      vtkwriter->SetInputData(out_VTKCube);
      vtkwriter->SetFileTypeToASCII();
      vtkwriter->SetFileName("ITKCubeMeshConvertedToVTK.ply");
      vtkwriter->Write();

      itkwriter->SetInput(out_ITKCube);
      itkwriter->SetFileTypeAsASCII();
      itkwriter->SetFileName("VTKCubeMeshConvertedToITK.obj");
      itkwriter->Write();
    }
    else if (ShapeCounter == 2){

      vtkwriter->SetInputData(out_VTKArch);
      vtkwriter->SetFileTypeToASCII();
      vtkwriter->SetFileName("ITKArchMeshConvertedToVTK.ply");
      vtkwriter->Write();

      itkwriter->SetInput(out_ITKArch);
      itkwriter->SetFileTypeAsASCII();
      itkwriter->SetFileName("VTKArchMeshConvertedToITK.obj");
      itkwriter->Write();
    }
    else if (ShapeCounter == 3){

      vtkwriter->SetInputData(out_VTKSphere);
      vtkwriter->SetFileTypeToASCII();
      vtkwriter->SetFileName("ITKSphereMeshConvertedToVTK.ply");
      vtkwriter->Write();

      itkwriter->SetInput(out_ITKSphere);
      itkwriter->SetFileTypeAsASCII();
      itkwriter->SetFileName("VTKSphereMeshConvertedToITK.obj");
      itkwriter->Write();
    }
    else if (ShapeCounter == 4){

      vtkwriter->SetInputData(out_VTKCone);
      vtkwriter->SetFileTypeToASCII();
      vtkwriter->SetFileName("ITKConeMeshConvertedToVTK.ply");
      vtkwriter->Write();

      itkwriter->SetInput(out_ITKCone);
      itkwriter->SetFileTypeAsASCII();
      itkwriter->SetFileName("VTKConeMeshConvertedToITK.obj");
      itkwriter->Write();
    }

      ++ShapeCounter;
  }

  return EXIT_SUCCESS;
}