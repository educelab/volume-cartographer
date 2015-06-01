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
  Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, double = -0.5);
  void update(Field&,int);
  bool isMoving();
  int size();
  int minIndex();
  Particle operator[](int);
  cv::Vec3f springForce(int);
 private:
  std::vector<Particle> _chain;
  double _spring_resting_x;
  double _spring_constant_k;
};

#endif
