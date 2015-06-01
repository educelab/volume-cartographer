#include "structureTensorParticleSim.h"

pcl::PointCloud<pcl::PointXYZRGB> returnPointCloud(std::vector<std::vector<pcl::PointXYZRGB> >,int,int);

pcl::PointCloud<pcl::PointXYZRGB> structureTensorParticleSim(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg volpkg, int gravity_scale, int threshold, int endOffset) {
  Field normal_field(&volpkg);
  Chain particle_chain(segPath);

  ////////////////////////////////////////////////////////////////////////////////
  int min_index = particle_chain.minIndex();
  int max_index = ((endOffset == -1)
                   ? (volpkg.getNumberOfSlices() - 3)
                   : (min_index + endOffset));
  int numslices = (max_index - min_index) + 1;
  int realIterations = int(ceil(numslices/threshold));
  std::vector<std::vector<pcl::PointXYZRGB> > VoV;

  for (int i = 0; i < realIterations; ++i) {
    std::vector<pcl::PointXYZRGB> tmp;
    for (int j = 0; j < particle_chain.size(); ++j) {
      pcl::PointXYZRGB point;
      point.x = -1;
      tmp.push_back(point);
    }
    VoV.push_back(tmp);
  }
  ////////////////////////////////////////////////////////////////////////////////
  
  for (int iteration = 0; particle_chain.isMoving() && iteration < 25000; ++iteration) {
    for (int i = 0; i < particle_chain.size(); ++i) {
      if (floor(particle_chain[i](0)) > max_index) {
        particle_chain[i].invalidate();
        continue;
      }

      int currentCell = int(((particle_chain[i](0)) - min_index/threshold));
      if (VoV[currentCell][i].x == -1) {
        uint32_t COLOR = 0x00777777;
        pcl::PointXYZRGB point;
        point.x = particle_chain[i](0);
        point.y = particle_chain[i](1);
        point.z = particle_chain[i](2);
        point.rgb = *reinterpret_cast<float*>(&COLOR);
        VoV[currentCell][i] = point;
      }
    }
    particle_chain.update(normal_field, gravity_scale);
    normal_field.clean();
  }
  
  return returnPointCloud(VoV, realIterations, particle_chain.size());
}


pcl::PointCloud<pcl::PointXYZRGB> returnPointCloud(std::vector<std::vector<pcl::PointXYZRGB> > storage, int realIterations, int chain_length) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height = realIterations;
  cloud.width = chain_length;
  cloud.points.resize(cloud.height * cloud.width);
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      cloud.points[j+(i*cloud.width)] = storage[i][j];
    }
  }
  return cloud;
}
