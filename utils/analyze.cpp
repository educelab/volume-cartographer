#include <stdio.h>
#include <opencv2/opencv.hpp>

int main( int argc, char *argv[] )
{
  if( argc < 2 )
  {
    fprintf( stderr, "Usage: %s in_file\n", argv[0] );
    return 1;
  }
  else if (argc < 3)
  {
    fprintf( stderr, "Error: No usage mode selected\n" );
    return 1;
  }
  
  const char *in_file = argv[1];

  // Load the input
  cv::Mat inImage;
  inImage = cv::imread( in_file, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );

  if (strcmp(argv[2],"size") == 0)
  {
    fprintf( stdout, "%d %d\n", inImage.cols, inImage.rows );
  }
  else if (strcmp(argv[2],"depth") == 0)
  {
    // 0 = CV_8U  - 8-bit unsigned
    // 1 = CV_8S  - 8-bit signed
    // 2 = CV_16U - 16-bit unsigned
    // 3 = CV_16S - 16-bit signed
    // 4 = CV_32S - 32-bit signed integers
    // 5 = CV_32F - 32-bit floating-point
    // 6 = CV_64F - 64-bit floating-point
    fprintf( stdout, "%d\n", inImage.depth() );
  }
  else
  {
    fprintf( stderr, "Error: Unknown usage mode\n" );
    return 1;
  }
    
return 0;
}