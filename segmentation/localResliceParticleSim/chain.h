// Chain object maintains a vector of points and their histories.
// step() updates the postitions of particles based on their normal
// vectors. Neighboring particles are kept in line with a "spring".

#ifndef _DEMO_CHAIN_
#define _DEMO_CHAIN_

#define DEFAULT_OFFSET -1

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "field.h"
#include "particle.h"

namespace DEMO {

class Chain {
public:
  Chain(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,VolumePkg*,int,int);
  void step(Field&);
  bool isMoving();
  void debug();
  pcl::PointCloud<pcl::PointXYZRGB> orderedPCD();

private:
  // History of the chain at each iteration
  std::list<std::vector<Particle> > _history;

  VolumePkg* _volpkg;

  // -- Chain Size Information -- //
  int _chain_length; // Number of particles in the chain & width of output PCD
  int _real_iterations; // Height of the output PCD To-Do: Do we need this?
  int _start_index; // Starting slice index
  int _target_index; // Target slice index
  int _threshold; // To-Do: What is this for now? We may not need this.
};

}

#endif
