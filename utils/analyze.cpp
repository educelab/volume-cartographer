#include <stdio.h>
#include <opencv2/opencv.hpp>

int main( int argc, char *argv[] )
{
  if( argc < 2 )
  {
    fprintf( stderr, "Usage: %s in_file\n", argv[0] );
    return 1;
  }
  
  const char *in_file = argv[1];

  cv::Mat inImage;

  // Load the input
  inImage = cv::imread( in_file, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );

  cv::Size s = inImage.size();
  fprintf( stdout, "%d %d\n", s.width, s.height );
    
return 0;
}