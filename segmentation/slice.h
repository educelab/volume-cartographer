#ifndef _SLICE_
#define _SLICE_

#include <opencv2/opencv.hpp>

class Slice {
public:
  Slice(cv::Mat,cv::Vec3f,cv::Vec3f);
  void scan();
  void debugDraw();
private:
  cv::Mat slice_;
  cv::Vec3f origin_;
  cv::Vec3f x_component_;
};

#endif
