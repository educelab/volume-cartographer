//
// Created by Media Team on 6/24/15.
//

#include "vc_defines.h"

#include "io/ply2itk.h"
#include "itk2vtk.h"

#include "vtkPLYWriter.h"
#include "itkMeshFileWriter.h"

int main( int argc, char* argv[] ) {

  std::string meshName = argv[1];

  // declare pointer to new Mesh object
  VC_MeshType::Pointer  inputMesh = VC_MeshType::New();

  int meshWidth = -1;
  int meshHeight = -1;

  // try to convert the ply to an ITK mesh
  if ( !volcart::io::ply2itkmesh( meshName, inputMesh, meshWidth, meshHeight ) ) {
    exit( EXIT_SUCCESS );
  };

  // Test the itk2vtk converter
  vtkPolyData *outputVTK = vtkPolyData::New();
  volcart::meshing::itk2vtk(inputMesh, outputVTK);

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