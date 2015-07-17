#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include "orderedPCDMesher.h"
#include "resamplePointCloud.h"
#include "greedyProjectionMeshing.h"

int main(int argc, char* argv[]) {
  if (argc < 2)
  {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " orderedFile.pcd" << std::endl;
    return (1);
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *inputCloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }

  std::string outfile = argv[1];
  //replace the extension
  int dot = outfile.find_last_of(".");
  outfile = outfile.substr(0,dot) + ".ply";
  
  volcart::meshing::orderedPCDMesher(inputCloud, outfile);

  // TEMP - test resampling
  pcl::PointCloud<pcl::PointXYZRGBNormal> output = volcart::meshing::resamplePointCloud(inputCloud);
  pcl::io::savePLYFile ("resampled.ply", output);
  //pcl::io::saveOBJFile ("resampled.ply", output);

  // TEMP - test greedy projection
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr resampled(&output);
  pcl::PolygonMesh mesh = volcart::meshing::greedyProjectionMeshing(resampled, 100, 7, 10);
  pcl::io::savePLYFile ("greedy.ply", mesh);
  pcl::io::saveOBJFile ("greedy.obj", mesh);

  // TEMP - test poisson
  

  exit(EXIT_SUCCESS);
}
