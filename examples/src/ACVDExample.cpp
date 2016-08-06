//
// Created by Seth Parker on 9/24/15.
//

#include "common/vc_defines.h"
#include "common/io/objWriter.h"
#include "common/io/ply2itk.h"
#include "meshing/itk2vtk.h"
#include "meshing/ACVD.h"

int main( int argc, char* argv[] ) {

  std::string meshName = argv[1];

  // declare pointer to new Mesh object
  VC_MeshType::Pointer  itkMesh = VC_MeshType::New();

  int meshWidth = -1;
  int meshHeight = -1;

  if ( !volcart::io::ply2itkmesh( meshName, itkMesh, meshWidth, meshHeight ) ) {
    exit( EXIT_SUCCESS );
  };

  vtkPolyData* vtkMesh = vtkPolyData::New();
  volcart::meshing::itk2vtk(itkMesh, vtkMesh);

  vtkPolyData* acvdMesh = vtkPolyData::New();
  volcart::meshing::ACVD(vtkMesh, acvdMesh, 10000);

  VC_MeshType::Pointer outputMesh = VC_MeshType::New();
  volcart::meshing::vtk2itk(acvdMesh, outputMesh);

  volcart::io::objWriter mesh_writer("acvd.obj", outputMesh);
  mesh_writer.write();

  return EXIT_SUCCESS;
}
