#ifndef _CHAIN_
#define _CHAIN_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "field.h"
#include "particle.h"

class Chain {
 public:
  Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,VolumePkg*,int,int,int,double = -0.5);
  void update(Field&);
  bool isMoving();
  cv::Vec3f springForce(int);
  pcl::PointCloud<pcl::PointXYZRGB> orderedPCD();

 private:
  std::list<std::vector<Particle> > _history;
  double _spring_constant_k;
  double _spring_resting_x;
  int _gravity_scale;
  int _chain_length;
  int _real_iterations;
  int _min_index;
  int _max_index;
  int _threshold;
};

#endif
