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
    std::cerr << argv[0] << " [input].pcd" << std::endl;
    return (1);
  }

  if ( argc < 2 ) {
    std::cerr << "ERROR: Not enough input arguments." << std::endl;
  }

  // Setup file paths
  std::string inputpath, outputpath;
  
  inputpath = argv[1];
  if(inputpath.substr(inputpath.find_last_of(".") + 1) != "pcd") {
    std::cerr << "ERROR: Input cloud file does not have expected .pcd extension." << std::endl;
    exit(EXIT_FAILURE);
  }

  outputpath = "cloud.pcd";

  // prepare point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (inputpath, *inputCloud) == -1) {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }
  pcl::io::savePCDFileASCII("cloud_zxy.pcd", *inputCloud); // Go ahead and save the original cloud

  pcl::PointCloud<pcl::PointXYZRGB>::iterator origPoint = inputCloud->begin();
  for (; origPoint != inputCloud->end(); ++origPoint) {
    pcl::PointXYZRGB newPoint;

    // Shift ZXY->XYZ:
    // new_X->old_Y, new_Y->old_Z, new_Z->old_X
    newPoint.x = origPoint->y;
    newPoint.y = origPoint->z;
    newPoint.z = origPoint->x;

    // RGB stays the same? To-Do: Ask Mike
    newPoint.rgb = origPoint->rgb;

    // Add it to our new cloud
    outputCloud->push_back(newPoint);
  }

  outputCloud->height = inputCloud->height;
  outputCloud->width = inputCloud->width;
  outputCloud->points.resize(outputCloud->height * outputCloud->width);

  printf("Writing point cloud to file...\n");
  pcl::io::savePCDFileASCII(outputpath, *outputCloud);
  printf("Point cloud saved.\n");

}