#include "slice.h"

#define WHITE 255
#define BGR_YELLOW cv::Scalar(0, 255, 255)
#define BGR_MAGENTA cv::Scalar(255, 0, 255)
#define COLOR_NORMAL BGR_YELLOW
#define COLOR_TANGENT BGR_MAGENTA

Slice::Slice(cv::Mat slice, cv::Vec3f origin, cv::Vec3f direction) {
  slice_ = slice;
  origin_ = origin;
  x_component_ = direction;
}

void Slice::scan() {
  cv::Mat grad_x;

  // second derivative gives us a nice void space
  // around the pages of the scroll
  cv::Mat temp_slice = slice_.clone();
  GaussianBlur(temp_slice, grad_x, cv::Size(3,3), 0);
  Sobel(grad_x, grad_x, CV_16S,1,0,3,1,0, cv::BORDER_DEFAULT);
  Sobel(grad_x, grad_x, CV_16S,1,0,3,1,0, cv::BORDER_DEFAULT);

  // when this is moved to production there needs to be
  // an if to make sure the pixel in question is black

  cv::Mat fill;
  grad_x *= 1./255;
  grad_x.convertTo(fill, CV_8UC1);

  // slice is easier to work with when it's just black and white
  floodFill(fill, cv::Point(fill.cols/2, fill.rows/2), WHITE);
  threshold(fill, fill, WHITE - 1, WHITE, cv::THRESH_BINARY);

  // display processed slice
  // will be moved to be a debug output later
  namedWindow("SCAN DEMO", cv::WINDOW_AUTOSIZE);
  imshow("SCAN DEMO", fill);
}

// debug draw uses the same normal and tangent coloring
// scheme to stay consistent with how we're reslicing
void Slice::debugDraw() {
  cv::Mat debug = slice_.clone();
  debug *= 1./255;
  debug.convertTo(debug, CV_8UC3);
  cvtColor(debug, debug, CV_GRAY2BGR);
  
  cv::Point imcenter(debug.cols/2, debug.rows/2);
  arrowedLine(debug, imcenter, imcenter + cv::Point(debug.cols/2 - 1, 0), COLOR_NORMAL);
  circle(debug, imcenter, 2, COLOR_TANGENT, -1);

  namedWindow("DEBUG DRAW", cv::WINDOW_AUTOSIZE);
  imshow("DEBUG DRAW", debug);
}
