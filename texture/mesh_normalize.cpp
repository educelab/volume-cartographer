#include <stdio.h>
#include "sliceIntersection.h"
#include "CMesh.h"
#include "CPlyHelper.h"
#include "CPoint.h"

int main( int argc, char *argv[] ) {

  if ( argc < 2 ) {
    std::cout << "Usage: vc_normalize mesh.ply" << std::endl;
    exit( -1 );
  }

  ChaoVis::CMesh thisMesh;
  std::string meshPath = argv[1];
  ChaoVis::CPlyHelper::ReadPlyFile( meshPath, thisMesh );
  std::cout << "Mesh file loaded..." << std::endl;

  // Get the min/max
  pcl::PointXYZRGBNormal p;
  int low = 255;
  int high = 0;
  for ( int i = 0; i < thisMesh.fPoints.size(); ++i ) {
    p = thisMesh.fPoints[i];
    if (( p.r < low ) && ( p.r > 0 )) {
      low = p.r;
    }
    if ( p.r > high ) {
      high = p.r;
    }
  }
  printf( "Lowest intensity: %d\n", low);
  printf( "Highest intensity: %d\n", high);

  // Normalize
  int newRGB;
  for ( int i = 0; i < thisMesh.fPoints.size(); ++i ) {
    p = thisMesh.fPoints[i];
    if ( p.r != 0 ) {
      newRGB = 0 + (p.r - low)*(255-0)/(high-low);
      thisMesh.fPoints[i].r = newRGB;
      thisMesh.fPoints[i].g = newRGB;
      thisMesh.fPoints[i].b = newRGB;
    }
  }

  printf( "writing result\n" );
  std::string outname = meshPath.substr(0,meshPath.find_last_of("."));
  ChaoVis::CPlyHelper::WritePlyFile( outname + "_norm.ply", thisMesh );

  exit( 0 );
}