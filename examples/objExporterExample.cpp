//
// Created by Media Team on 6/24/15.
//

#include "vc_defines.h"

#include "io/objWriter.h"
#include "io/ply2itk.h"

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

  // pointID == point's position in 1D list of points
  // [meshX, meshY] == point's position if list was a 2D matrix
  // [u, v] == point's position in the output matrix
  unsigned long pointID, meshX, meshY;
  double u, v;
  std::map<double, cv::Vec2d> uvMap;

  // Initialize iterators
  VC_PointsInMeshIterator point = inputMesh->GetPoints()->Begin();
  std::cerr << "Calculating UV positions..." << std::endl;
  while ( point != inputMesh->GetPoints()->End() ) {

    pointID = point.Index();

    // Calculate the point's [meshX, meshY] position based on its pointID
    meshX = pointID % meshWidth;
    meshY = (pointID - meshX) / meshWidth;

    // Calculate the point's UV position
    u =  (double) meshX / (double) meshWidth;
    v =  (double) meshY / (double) meshHeight;

    // OBJ UV coordinates start in the bottom-left of an image, but we're calculating them from the top-left
    // Subtract v from 1.0 to account for this
    cv::Vec2d uv( u, 1.0 - v );

    // Add the uv coordinates into our map at the point index specified
    uvMap.insert( {pointID, uv} );

    ++point;
  }
  std::cerr << std::endl;

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