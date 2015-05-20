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

  // Return the size of the image to the console as "width height"
  if (strcmp(argv[2],"size") == 0)
  {
    fprintf( stdout, "%u %u\n", inImage.cols, inImage.rows );
  }
  // Return the depth of the image to the console as OpenCV int ID
  // 0 = CV_8U  - 8-bit unsigned
  // 1 = CV_8S  - 8-bit signed
  // 2 = CV_16U - 16-bit unsigned
  // 3 = CV_16S - 16-bit signed
  // 4 = CV_32S - 32-bit signed integers
  // 5 = CV_32F - 32-bit floating-point
  // 6 = CV_64F - 64-bit floating-point
  else if (strcmp(argv[2],"depth") == 0)
  {
    fprintf( stdout, "%d\n", inImage.depth() );
  }
  // Return the minimum and maximum intensity values of the image to console as "min max"
  else if (strcmp(argv[2],"minmax") == 0)
  {
    double min, max;
    cv::minMaxLoc(inImage, &min, &max);
    fprintf( stdout, "%f %f\n", min, max );
  }
  // Return the size, depth, and min-max info to the console as "width height depth min max"
  else if (strcmp(argv[2],"all") == 0)
  {
    double min, max;
    cv::minMaxLoc(inImage, &min, &max);
    fprintf( stdout, "%d %d %d %f %f\n", inImage.cols, inImage.rows, inImage.depth(), min, max );
  }
  else
  {
    fprintf( stderr, "Error: Unknown usage mode\n" );
    return 1;
  }
    
return 0;
}