#ifndef _FIELD_
#define _FIELD_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "volumepkg.h"

class Field {
 public:
  Field(VolumePkg*);
  ~Field();
  void clean();
  cv::Vec3f interpolate_at(cv::Vec3f, int);
 private:
  VolumePkg* _volpkg;
  cv::Vec3f*** _field;
  int _blocksize;
  std::set<int> _indexes_used_since_last_clean;
  void loadSlice(int);
};

#endif
