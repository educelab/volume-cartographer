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

cv::Vec3f Slice::findNextPosition() {
  // second derivative gives us a nice void space
  // around the pages of the scroll
  cv::Mat temp_slice = slice_.clone();
  cv::Mat grad_x;
  GaussianBlur(temp_slice, grad_x, cv::Size(3,3), 0);
  Sobel(grad_x, grad_x, CV_16S,1,0,3,1,0, cv::BORDER_DEFAULT);
  Sobel(grad_x, grad_x, CV_16S,1,0,3,1,0, cv::BORDER_DEFAULT);

  // when this is moved to production there needs to be
  // an if to make sure the pixel in question is black

  cv::Mat fill;
  grad_x *= 1./255;
  grad_x.convertTo(fill, CV_8UC1);


  // slice is easier to work with when it's just black and white
  cv::Point center(fill.cols/2, fill.rows/2);
  floodFill(fill, center, WHITE);
  threshold(fill, fill, WHITE - 1, WHITE, cv::THRESH_BINARY);

  // find the new position in the reslice
  cv::Point newPosition;
  for (int xoffset = 0; xoffset < fill.cols/2; ++xoffset) {
    cv::Point positive_offset(xoffset,1);
    if (fill.at<uchar>(center + positive_offset)) {
      newPosition = center + positive_offset;
      break;
    }

    cv::Point negative_offset(-xoffset,1);
    if (fill.at<uchar>(center + negative_offset)) {
      newPosition = center + negative_offset;
      break;
    }
  }

  cv::Vec3f xyzPosition = origin_ + (newPosition.x * x_component_ + newPosition.y * cv::Vec3f(0,0,1));

  std::cout << xyzPosition << std::endl;

  return xyzPosition;
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
