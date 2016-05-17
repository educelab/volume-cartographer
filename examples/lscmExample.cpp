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

  // Compute parameterization
  volcart::texturing::lscm lscm( inputMesh );
  lscm.compute();

  // Get uv map
  volcart::UVMap uvMap = lscm.getUVMap();
  int width = 800;
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