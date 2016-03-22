#include <iostream>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include "volumepkg.h"
#include "vc_datatypes.h"
#include "vc_defines.h"

namespace fs = boost::filesystem;

VolumePkg* _volpkg;
cv::Mat _texture, _perPixelMask, _currentSlice;
volcart::PerPixelMap _perPixelMap;

cv::Vec3d lookup2Dto3D( int x, int y);
cv::Mat   loadConverted( int slice_index );

static void onMouse( int event, int x, int y, int, void* ) {
  std::cerr << "callback" << std::endl;
  // Filter for clicks
  if ( event != cv::EVENT_LBUTTONDOWN )
    return;

  // Check the mask for to make sure we have a lookup
  int maskVal = _perPixelMask.at< unsigned char >(y,x);
  if ( maskVal == 0)
    return;

  // Get the 3D position
  cv::Vec3d pos3D = lookup2Dto3D( y, x );

  // Load the slice
  _currentSlice = loadConverted( cvRound(pos3D(2)) );

  // Draw a circle
  cv::circle( _currentSlice, cvPoint( pos3D(0), pos3D(1) ), 3, cv::Scalar(0,0,255), -1 );

  // Show the updated
  cv::imshow("slice", _currentSlice);
};

int main ( int argc, char* argv[] ) {

  _volpkg = new VolumePkg( argv[1] );
  _volpkg->setActiveSegmentation( argv[2] );
  fs::path segPath = fs::canonical(fs::path(_volpkg->getMeshPath()).parent_path());

  // Load the initial stuff
  _texture = cv::imread( segPath.string() + "/textured.png" );
  std::cout << "texture: " << _texture.cols << ", " << _texture.rows << std::endl;

  _perPixelMask = cv::imread( segPath.string() + "/PerPixelMask.png", CV_LOAD_IMAGE_GRAYSCALE );
  std::cout << "mask: " << _perPixelMask.cols << ", " << _perPixelMask.rows << std::endl;

  std::string ppm_path = segPath.string() + "/PerPixelMapping.yml.gz";
  _perPixelMap.read( ppm_path );

  _currentSlice = loadConverted(0);

  // Windows
  cv::namedWindow( "texture", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  cv::namedWindow( "slice", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED );

  cv::setMouseCallback( "texture", onMouse, 0 );

  while( true ) {
    cv::imshow("image", _texture);
    cv::imshow("slice", _currentSlice);

    int c = cv::waitKey(0);
    if ( c != -1 ) break;
  }

  return EXIT_SUCCESS;
}

cv::Vec3d lookup2Dto3D( int y, int x ) {
  return cv::Vec3d( _perPixelMap(y,x)(0), _perPixelMap(y,x)(1), _perPixelMap(y,x)(2) );
}

cv::Mat loadConverted(int slice_index) {
  cv::Mat tmp = _volpkg->volume().getSliceDataCopy( slice_index );
  tmp /= 255.0;
  tmp.convertTo(tmp, CV_8UC3);
  cv::cvtColor(tmp, tmp, CV_GRAY2BGR);

  return tmp;
};