#ifndef _SLICE_
#define _SLICE_

#include <opencv2/opencv.hpp>


// defines for uniform indexing
// should be moved somewhere more global
#define VC_INDEX_X 0
#define VC_INDEX_Y 1
#define VC_INDEX_Z 2

#define VC_DIRECTION_I cv::Vec3f(1,0,0)
#define VC_DIRECTION_J cv::Vec3f(0,1,0)
#define VC_DIRECTION_K cv::Vec3f(0,0,1)

// slice class used for keeping track of 2d sections
// in 3d space
class Slice {
public:
  Slice(cv::Mat,cv::Vec3f,cv::Vec3f,cv::Vec3f);
  cv::Vec3f findNextPosition();
  void debugDraw(bool=true);
  void debugFloodFill();
  cv::Mat mat();
private:
  cv::Mat slice_;
  cv::Vec3f origin_;
  cv::Vec3f x_component_;
  cv::Vec3f y_component_;
};

#endif
