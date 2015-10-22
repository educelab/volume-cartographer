//
// Created by Media Team on 6/24/15.
//

#include "vc_defines.h"
#include "vc_datatypes.h"

#include "io/objWriter.h"
#include "io/ply2itk.h"

#include "simpleUV.h"

int main( int argc, char* argv[] ) {

  std::string meshName = argv[1];
  cv::Mat uvImg = cv::imread( argv[2], CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );

  // declare pointer to new Mesh object
  VC_MeshType::Pointer  inputMesh = VC_MeshType::New();

  int meshWidth = -1;
  int meshHeight = -1;

  // try to convert the ply to an ITK mesh
  if ( !volcart::io::ply2itkmesh( meshName, inputMesh, meshWidth, meshHeight ) ) {
    exit( EXIT_SUCCESS );
  };

  // Generate UV Map
  volcart::UVMap uvMap = volcart::texturing::simpleUV(inputMesh, meshWidth, meshHeight);

  volcart::io::objWriter mesh_writer;
  mesh_writer.setPath( "nothing" );
  mesh_writer.setUVMap( uvMap );
  mesh_writer.setTexture( uvImg );

  // mesh_writer.write() runs validate() as well, but this lets us handle a mesh that can't be validated.
  if ( mesh_writer.validate() )
    mesh_writer.write();
  else {
    mesh_writer.setPath( "output.obj" );
    mesh_writer.setMesh( inputMesh );
    mesh_writer.write();
  }


  return EXIT_SUCCESS;
}