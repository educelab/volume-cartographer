#ifndef _FIELD_
#define _FIELD_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "volumepkg.h"
#include "slice.h"

#define SLICE_DIR cv::Vec3f(0,0,1)

class Field {
public:
  Field(VolumePkg*);
  unsigned short interpolate_at(cv::Vec3f);
  Slice reslice(cv::Vec3f,cv::Vec3f,int = 64, int = 64);
private:
  VolumePkg* _volpkg;
  std::vector<cv::Mat> _field;
};

#endif
