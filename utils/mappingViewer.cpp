#include <iostream>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include "volumepkg.h"
#include "vc_datatypes.h"
#include "vc_defines.h"

namespace fs = boost::filesystem;

VolumePkg* _volpkg;
cv::Mat _texture, _perPixelMask;
volcart::Texture::PerPixelMap _perPixelMap;

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

  cv::FileStorage file( segPath.string() + "/PerPixelMapping.yml.gz", cv::FileStorage::READ );
  cv::FileNode map = file["PerPixelMapping"];

  int width  = (int) map["cols"];
  int height = (int) map["rows"];
  _perPixelMap = volcart::Texture::PerPixelMap( height, width, cv::Vec6d(0,0,0,0,0,0) );

  cv::FileNodeIterator dbl = map["data"].begin();
  for( int y = 0; y < height; ++y ) {
    for( int x = 0; x < width; ++x ) {

      cv::Vec6d v;
      for( int n = 0; n < 6; ++n, ++dbl ) {
        v(n) = (double)(*dbl);
      }

      _perPixelMap(y,x) = v;
    }
  }

  file.release();
  std::cout << "map: " << _perPixelMap.cols << ", " << _perPixelMap.rows << std::endl;

  for( int y = 0; y < height; ++y ) {
    for( int x = 0; x < width; ++x ) {
      int val = _perPixelMask.at< unsigned char >(y,x);
      if ( val == 255 ) {
        std::cout << x << ", " << y << " | " << val << " | " << _perPixelMap(y,x) << std::endl;
      }
    }
  }

  return EXIT_SUCCESS;
}

cv::Vec3d lookup2Dto3D( int x, int y ) {

}