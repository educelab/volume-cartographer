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

#include "io/ply2itk.h"
#include "itk2vtk.h"
#include "shapes.h"

#include "vtkPLYWriter.h"
#include "itkMeshFileWriter.h"

int main( int argc, char* argv[] ) {

  volcart::shapes::Plane inputMesh;

  // Test the itk2vtk converter
  vtkPolyData *outputVTK = vtkPolyData::New();
  volcart::meshing::itk2vtk(inputMesh.itkMesh(), outputVTK);

  vtkPLYWriter *vtkwriter = vtkPLYWriter::New();
  vtkwriter->SetInputData(outputVTK);
  vtkwriter->SetFileTypeToASCII();
  vtkwriter->SetFileName("vtk.ply");
  vtkwriter->Write();

  // Test the vtk2itk converter
  VC_MeshType::Pointer outputITK = VC_MeshType::New();
  volcart::meshing::vtk2itk(outputVTK, outputITK);

  itk::MeshFileWriter<VC_MeshType>::Pointer itkwriter = itk::MeshFileWriter<VC_MeshType>::New();
  itkwriter->SetInput(outputITK);
  itkwriter->SetFileTypeAsASCII();
  itkwriter->SetFileName("itk.obj");
  itkwriter->Write();

  return EXIT_SUCCESS;
}