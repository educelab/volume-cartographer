//
// Created by Media Team on 6/24/15.
//

#include "io/io_objExport.h"
#include "plyHelper.h"
#include <unordered_map>

#include <itkMesh.h>
#include <itkTriangleCell.h>

int main( int argc, char* argv[] ) {

  std::string meshName = argv[1];
  cv::Mat uvImg = cv::imread( argv[2] );

  // define ITK classes
  typedef itk::Vector< double, 3 >  PixelType;  // A vector to hold the normals along with the points of each vertex in the mesh
  const unsigned int Dimension = 3;   // Need a 3 Dimensional Mesh
  typedef itk::Mesh< PixelType, Dimension >   MeshType;

  typedef MeshType::PointsContainer::ConstIterator   PointsInMeshIterator;

  // declare pointer to new Mesh object
  MeshType::Pointer  inputMesh = MeshType::New();

  int meshWidth = -1;
  int meshHeight = -1;

  // try to convert the ply to an ITK mesh
  if ( !ply2itkmesh( meshName, inputMesh, meshWidth, meshHeight ) ) {
    exit( EXIT_SUCCESS );
  };

  // pointID == point's position in 1D list of points
  // [meshX, meshY] == point's position if list was a 2D matrix
  // [u, v] == point's position in the output matrix
  unsigned long pointID, meshX, meshY;
  double u, v;
  std::unordered_map<unsigned long, cv::Vec2d> uvMap;

  // Initialize iterators
  PointsInMeshIterator point = inputMesh->GetPoints()->Begin();
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

  volcart::io::objWriter objWriter( "mesh.obj", inputMesh, uvMap, uvImg );

  objWriter.write();

  return EXIT_SUCCESS;
}