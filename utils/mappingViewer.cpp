#include <iostream>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include "volumepkg.h"
#include "vc_datatypes.h"
#include "vc_defines.h"

namespace fs = boost::filesystem;

VolumePkg* _volpkg;
cv::Mat _texture, _perPixelMask;
volcart::PerPixelMap _perPixelMap;

cv::Vec3d lookup2Dto3D( int x, int y);

int main ( int argc, char* argv[] ) {

  _volpkg = new VolumePkg( argv[1] );
  _volpkg->setActiveSegmentation( argv[2] );
  fs::path segPath = fs::canonical(fs::path(_volpkg->getMeshPath()).parent_path());

  // Load the two simple things

  _texture = cv::imread( segPath.string() + "/textured.png" );
  std::cout << "texture: " << _texture.cols << ", " << _texture.rows << std::endl;

  _perPixelMask = cv::imread( segPath.string() + "/PerPixelMask.png", CV_LOAD_IMAGE_GRAYSCALE );
  std::cout << "mask: " << _perPixelMask.cols << ", " << _perPixelMask.rows << std::endl;

  std::string ppm_path = segPath.string() + "/PerPixelMapping.yml.gz";
  _perPixelMap.read( ppm_path );

  for( int y = 0; y < _perPixelMap.height(); ++y ) {
    for( int x = 0; x < _perPixelMap.width(); ++x ) {
      int val = _perPixelMask.at< unsigned char >(y,x);
      if ( val == 255 ) {
        std::cout << x << ", " << y << " | " << lookup2Dto3D( y, x ) << std::endl;
      }
    }
  }

  return EXIT_SUCCESS;
}

cv::Vec3d lookup2Dto3D( int y, int x ) {
  return cv::Vec3d( _perPixelMap(y,x)(0), _perPixelMap(y,x)(1), _perPixelMap(y,x)(2) );
}