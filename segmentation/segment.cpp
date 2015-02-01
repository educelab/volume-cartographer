#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "volumepkg.h"
#include "modules/structureTensorParticleSim.h"

// Landmarks
std::vector<Particle> landmark_chain;

// File paths
std::string pathLocation = "";
std::string volpkgLocation = "";
std::string outputName = "";

// Options
int gravity_scale = -1;
int threshold = -1;
int endSlice = -1;

int main(int argc, char* argv[]) {
  std::cout << "vc_simulation" << std::endl;
  if (argc < 5) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " {--gravity [1-10] --threshold [1-10] --endAfter [value]} --path [Path.txt] --volpkg [volpkgpath]" << std::endl;
    exit(EXIT_FAILURE);
  }

  // get gravity scale value from command line
  pcl::console::parse_argument (argc, argv, "--gravity", gravity_scale);
  if (gravity_scale == -1) {
    std::cout << "No Gravity Scale value given, defaulting to 2" << std::endl;
    gravity_scale = 2;
  }

  pcl::console::parse_argument (argc, argv, "--threshold", threshold);
  if (threshold == -1) {
    std::cout << "No Distance Threshold value given, defaulting to 1" << std::endl;
    threshold = 1;
  }
  
  pcl::console::parse_argument (argc, argv, "--endAfter", endSlice);

  pcl::console::parse_argument (argc, argv, "--path", pathLocation);
  if (pathLocation == "") {
    std::cerr << "ERROR: Incorrect/missing path location!" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  pcl::console::parse_argument (argc, argv, "--volpkg", volpkgLocation);
  if (volpkgLocation == "") {
    std::cerr << "ERROR: Incorrect/missing volpkg location!" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  // generate output name from path file
  outputName = pathLocation.substr(pathLocation.find_last_of("/\\")+1);
  outputName = outputName.substr(0,outputName.find_last_of("."));

  // Load volume package
  VolumePkg volpkg(volpkgLocation);

  // Run segmentation
  pcl::PointCloud<pcl::PointXYZRGB> segmentedCloud;
  segmentedCloud = structureTensorParticleSim(pathLocation, volpkg, gravity_scale, threshold, endSlice);

  // Save point cloud
  printf("Writing point cloud to file...\n");
  pcl::io::savePCDFileASCII(outputName + "_segmented.pcd", segmentedCloud);

  printf("Segmentation complete!\n");
  exit(EXIT_SUCCESS);
}