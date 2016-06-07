//
// Created by Seth Parker on 6/24/15.
//

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "shapes.h"
#include "volumepkg.h"

#include "io/plyWriter.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"

#include "lscm.h"
#include "compositeTextureV2.h"

#include <vtkPLYReader.h>

int main( int argc, char* argv[] ) {

  VolumePkg vpkg( argv[1] );
  vpkg.setActiveSegmentation( argv[2] );

  // Read the mesh
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( "decim.ply" );
  reader->Update();
  VC_MeshType::Pointer inputMesh = VC_MeshType::New();
  volcart::meshing::vtk2itk( reader->GetOutput(), inputMesh );

  std::string meshName = vpkg.getMeshPath();

  // declare pointer to new Mesh object
  VC_MeshType::Pointer  input = VC_MeshType::New();
  int meshWidth = -1;
  int meshHeight = -1;

  // try to convert the ply to an ITK mesh
  if (!volcart::io::ply2itkmesh(meshName, input, meshWidth, meshHeight)){
    exit( -1 );
  };

  // Compute parameterization
  volcart::texturing::lscm lscm( input );
  lscm.compute();

  // Get uv map
  volcart::UVMap uvMap = lscm.getUVMap();
  int width = std::ceil( uvMap.ratio().width );
  int height = std::ceil( (double) width / uvMap.ratio().aspect );

  volcart::texturing::compositeTextureV2 compText(inputMesh, vpkg, uvMap, 5, width, height);

  volcart::io::objWriter mesh_writer;
  mesh_writer.setPath( "lscm.obj" );
  mesh_writer.setMesh( inputMesh );
  mesh_writer.setTexture( compText.texture().getImage(0) );
  mesh_writer.setUVMap( compText.texture().uvMap() );
  mesh_writer.write();


  return EXIT_SUCCESS;
}