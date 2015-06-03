#include "structureTensorParticleSim.h"
// Describe overview of algorithm*
pcl::PointCloud<pcl::PointXYZRGB> structureTensorParticleSim(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg volpkg, int gravity_scale, int threshold, int endOffset) {
  // Load the surface normal field from volpkg
  // Field object handles which files will be loaded and when
  Field normal_field(&volpkg);
  // Organize the points into a connected chain
  Chain particle_chain(segPath, &volpkg, gravity_scale, threshold, endOffset);
  
  // What does this loop define for a stop condition?*
  for (int iteration = 0; particle_chain.isMoving() && iteration < 25000; ++iteration) {
    // Make the chain take one iteration forward
    particle_chain.update(normal_field); // Change .update to makeStep or something similar*
    // Unload unused normal vectors to free memory
    normal_field.clean();
  }
  
  return particle_chain.orderedPCD();
}
