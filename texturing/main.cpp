// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "io/ply2itk.h"
#include "rayTrace.h"

#include "texturingUtils.h"
#include <itkRGBPixel.h>
#include "UPointMapping.h"


int main(int argc, char* argv[]) {
  if ( argc < 6 ) {
    std::cout << "Usage: vc_texture2 volpkg seg-id radius texture-method sample-direction tracing-direction" << std::endl;
    std::cout << "Texture methods: " << std::endl;
    std::cout << "      0 = Intersection" << std::endl;
    std::cout << "      1 = Non-Maximum Suppression" << std::endl;
    std::cout << "      2 = Maximum" << std::endl;
    std::cout << "      3 = Minimum" << std::endl;
    std::cout << "      4 = Median w/ Averaging" << std::endl;
    std::cout << "      5 = Median" << std::endl;
    std::cout << "      6 = Mean" << std::endl;
    std::cout << std::endl;
    std::cout << "Sample Direction: " << std::endl;
    std::cout << "      0 = Omni" << std::endl;
    std::cout << "      1 = Positive" << std::endl;
    std::cout << "      2 = Negative" << std::endl;
    std::cout << "Ray Tracing Direction: " << std::endl;
    std::cout << "      0 = Clockwise" << std::endl;
    std::cout << "      1 = Counterclockwise" << std::endl;
    exit( -1 );
  }

  VolumePkg vpkg = VolumePkg( argv[ 1 ] );
  std::string segID = argv[ 2 ];
  if (segID == "") {
    std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
    exit(EXIT_FAILURE);
  }
  if ( vpkg.getVersion() < 2.0) {
    std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
    exit(EXIT_FAILURE);
  }
  vpkg.setActiveSegmentation(segID);
  std::string meshName = vpkg.getMeshPath();
  double radius, minorRadius;
  radius = atof( argv[ 3 ] );
  if((minorRadius = radius / 3) < 1) minorRadius = 1;

  int aFindBetterTextureMethod = atoi( argv[ 4 ] );
  EFilterOption aFilterOption = ( EFilterOption )aFindBetterTextureMethod;

  int aSampleDir = atoi( argv[ 5 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
  EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

  int aTraceDir = atoi( argv[ 6 ] ); //rayTracingDirection (0=clockwise, 1=counterclockwise)

  // declare pointer to new Mesh object
  VC_MeshType::Pointer mesh = VC_MeshType::New();
  int meshWidth = -1;
  int meshHeight = -1;

  if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
    exit(-1);
  }

  // pointID == point's position in 1D list of points
  // [meshX, meshY] == point's position if list was a 2D matrix
  // [u, v] == point's position in the output matrix
  unsigned long pointID, meshX, meshY;
  double u, v;

  // Load the slices from the volumepkg
  std::vector< cv::Mat > aImgVol;

  /*  This function is a hack to avoid a refactoring the texturing
      methods. See Issue #12 for more details. */
  // Setup
  int meshLowIndex = (int) mesh->GetPoint(0)[2];
  int meshHighIndex = meshLowIndex + meshHeight;
  int aNumSlices = vpkg.getNumberOfSlices();

  int bufferLowIndex = meshLowIndex - (int) radius;
  if (bufferLowIndex < 0) bufferLowIndex = 0;

  int bufferHighIndex = meshHighIndex + (int) radius;
  if (bufferHighIndex >= vpkg.getNumberOfSlices()) {
    bufferHighIndex = vpkg.getNumberOfSlices();
  }

  // Slices must be loaded into aImgVol at the correct index: slice 005 == aImgVol[5]
  // To avoid loading the whole volume, pad the beginning indices with 1x1 null mats
  cv::Mat nullMat = cv::Mat::zeros(1, 1, CV_16U);
  for ( int i = 0; i < bufferLowIndex; ++i ) {
    std::cout << "\rLoading null buffer slices: " << i + 1 << "/" << bufferLowIndex << std::flush;
    aImgVol.push_back( nullMat.clone() );
  }
  std::cout << std::endl;

  // Load the actual volume into a tempVol with a buffer of nRadius
  for ( int i = bufferLowIndex; i < bufferHighIndex; ++i ) {
    std::cout << "\rLoading real slices: " << i - bufferLowIndex + 1 << "/" << bufferHighIndex - bufferLowIndex << std::flush;
    aImgVol.push_back( vpkg.getSliceData( i ).clone() );
  }
  std::cout << std::endl;

  // paramaters needed for rayTrace function
  int rayWidth = -1;
  int rayHeight = -1;
  std::map<int, cv::Vec2d> uvMap; // mapping to XY pixel positions in output texture
  std::vector<cv::Vec6f> resampledPoints;

  resampledPoints = volcart::meshing::rayTrace(mesh, aTraceDir, rayWidth, rayHeight, uvMap);

  cv::Mat outputTexture = cv::Mat::zeros( rayHeight, rayWidth, CV_16UC1 );

  for(int i = 0; i < resampledPoints.size(); ++i) {
    cv::Vec3f pt_pos(resampledPoints[i](0), resampledPoints[i](1), resampledPoints[i](2));
    cv::Vec3f pt_norm(resampledPoints[i](3), resampledPoints[i](4), resampledPoints[i](5));
    cv::Vec2f uv = uvMap.find(i)->second; // To-Do: Make sure this actually finds a point

    double color = textureWithMethod( pt_pos,
                                      pt_norm,
                                      aImgVol,
                                      aFilterOption,
                                      radius,
                                      minorRadius,
                                      0.5,
                                      aDirectionOption);
    
    outputTexture.at < unsigned short > (uv[1], uv[0]) = (unsigned short) color;
  }
  
  vpkg.saveTextureData(outputTexture);

  return 0;
} // main
