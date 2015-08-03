//
// Created by Media Team on 6/24/15.
//

#include "vc_defines.h"

#include "io/ply2itk.h"
#include "itk2vtk.h"

#include "vtkPLYWriter.h"

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

  vtkPolyData *outputMesh = vtkPolyData::New();

  volcart::meshing::itk2vtk(inputMesh, outputMesh);

  vtkPLYWriter *writer = vtkPLYWriter::New();
  writer->SetInputData(outputMesh);
  writer->SetFileTypeToASCII();
  writer->SetFileName("vtk.ply");
  writer->Write();

  return EXIT_SUCCESS;
}