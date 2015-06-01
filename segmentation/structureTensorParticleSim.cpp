#include "structureTensorParticleSim.h"

pcl::PointCloud<pcl::PointXYZRGB> structureTensorParticleSim(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg volpkg, int gravity_scale, int threshold, int endOffset) {
  Field normal_field(&volpkg);
  Chain particle_chain(segPath, &volpkg, gravity_scale, threshold, endOffset);

  for (int iteration = 0; particle_chain.isMoving() && iteration < 25000; ++iteration) {
    particle_chain.update(normal_field);
    normal_field.clean();
  }
  
  return particle_chain.orderedPCD();
}
