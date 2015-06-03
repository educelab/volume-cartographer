// Overview of this object*

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
  // History of the chain at each iteration
  std::list<std::vector<Particle> > _history;
  // Parameters for calculating the spring effects
  double _spring_constant_k;
  double _spring_resting_x;
  // Limits the effect of the normal vector
  int _gravity_scale; // To-Do: Rename. Redefine how this is used

  // -- Chain Size Information -- //
  int _chain_length; // Number of particles in the chain & width of output PCD
  int _real_iterations; // Height of the output PCD To-Do: Do we need this?
  int _min_index; // Starting slice index *To-do: Rename?
  int _max_index; // Target slice index
  int _threshold; // To-Do: What is this for now? We may not need this.
};

#endif
