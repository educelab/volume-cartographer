//
// Created by Seth Parker on 6/24/15.
//

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"

#include "io/plyWriter.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"

#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>

#include "abf.h"
#include "compositeTextureV2.h"

int main( int argc, char* argv[] ) {

  VolumePkg vpkg( argv[1] );

  // Read the mesh
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( "decim.ply" );
  reader->Update();
  VC_MeshType::Pointer inputMesh = VC_MeshType::New();
  volcart::meshing::vtk2itk( reader->GetOutput(), inputMesh );

  // ABF flattening
  volcart::texturing::abf abf(inputMesh);
  abf.compute();
  VC_MeshType::Pointer abfMesh = abf.getMesh();
  volcart::io::objWriter abfWriter( "abf.obj", abfMesh );
  abfWriter.write();

//  // Convert soft body to itk mesh
//  volcart::texturing::compositeTextureV2 result(inputMesh, vpkg, uvMap, 7, (int) aspect_width, (int) aspect_height);
//  volcart::io::objWriter objwriter("cloth.obj", inputMesh, result.texture().uvMap(), result.texture().getImage(0));
//  objwriter.write();


  return EXIT_SUCCESS;
}