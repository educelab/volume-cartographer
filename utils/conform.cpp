#include <stdio.h>
#include <opencv2/opencv.hpp>

int main( int argc, char *argv[] )
{
  if( argc < 3 )
  {
    fprintf( stderr, "Usage: %s in_file out_file\n", argv[0] );
    return 1;
  }
  
  const char *in_file = argv[1];
  const char *out_file = argv[2];

  cv::Mat inImage, workImage, outImage;

  // Load the input
  inImage = cv::imread( in_file, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );

  // Remap 8 bit values to 16 bit
  if ( inImage.depth() == CV_8U ) {
      int minVal, maxVal;
      minVal = 0;
      maxVal = 255;
      inImage.convertTo( workImage, CV_16U, 65535.0/(maxVal - minVal), -minVal * 65535.0/(maxVal - minVal));
  // TODO: need to account for CV_8S and 32-bit images
  } else {
      inImage.copyTo( workImage );
  }

  // Convert colorspace to grayscale
  if ( outImage.channels() > 1 ) {
      cv::cvtColor( workImage, outImage, CV_BGR2GRAY ); // OpenCV use BGR to represent color image
  } else {
      workImage.copyTo( outImage );
  }

  cv::imwrite( out_file, outImage );
    
return 0;
}