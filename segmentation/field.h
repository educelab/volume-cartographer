// Field object maintains a sliding window of loaded normal vectors in the volume
// Normals are read from disk when they're needed and can be deleted with clean()
// to keep the memory used to a minimum.

#ifndef _FIELD_
#define _FIELD_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "volumepkg.h"

#define RESLICE_WIDTH 64

class Field {
 public:
  Field(VolumePkg*);
  //  ~Field();
  //  void clean();
  unsigned short interpolate_at(cv::Vec3f);
  cv::Mat reslice(cv::Vec3f,cv::Vec3f);
 private:
  VolumePkg* _volpkg;
  std::vector<cv::Mat> _field;
  // Largest dimension of the CT slices
  //  int _blocksize;
  // Slices not in this set will be removed by clean()
  //  std::set<int> _indexes_used_since_last_clean;
  //  void loadSlice(int); // To-Do: Is this name clear enough?
};

#endif
