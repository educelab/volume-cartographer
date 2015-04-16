// txt2pcd
// Used to convert legacy VC Path txt's to PCL clouds
// Created: 04/16/2015
// Last Modified: 04/16/2015

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
  
int main(int argc, char* argv[]) {
  if (argc < 2)
  {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " [pathFile].txt {outputName.pcd}" << std::endl;
    return (1);
  }

  // Setup file paths
  std::string outputName, landmark_path;
  
  landmark_path = argv[1];
  if(landmark_path.substr(landmark_path.find_last_of(".") + 1) != "txt") {
    std::cerr << "ERROR: Path file does not have expected txt extension." << std::endl;
    exit(EXIT_FAILURE);
  }

  if(argv[2] != NULL){
    outputName = argv[2];
    if(outputName.substr(outputName.find_last_of(".") + 1) != "pcd") {
      outputName += ".pcd";
    }
  } else {
    outputName = "cloud.pcd";
  }

  // prepare point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  // read landmarks file
  std::ifstream landmarks_file;
  landmarks_file.open(landmark_path);
  if (landmarks_file.fail()) {
    std::cout << "ERROR: Path file could not be opened." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create chain from landmarks file
  while (!landmarks_file.eof()) {
    double index, a, b;
    pcl::PointXYZRGB point;
    landmarks_file >> index >> a >> b;
    // vc point clouds store voxel indices in a different order than the original slices
    // point clouds have format: cloud.xyz = volume.zxy
    point.x = index;
    point.y = a;
    point.z = b;
    outputCloud->push_back(point);
  }

  landmarks_file.close();

  printf("Writing point cloud to file...\n");
  pcl::io::savePCDFileASCII(outputName, *outputCloud);
  printf("Point cloud saved.\n");

}