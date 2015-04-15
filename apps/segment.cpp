#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/conditional_removal.h>

#include "volumepkg.h"
#include "structureTensorParticleSim.h"

// Landmarks
std::vector<Particle> landmark_chain;

// File paths
std::string segID = "";
std::string volpkgLocation = "";
std::string outputName = "";

// Options
int gravity_scale = -1;
int threshold = -1;
double startIndex = -1.0;
int stopOffset = -1;

int main(int argc, char* argv[]) {
  std::cout << "vc_segment" << std::endl;
  if (argc < 5) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " {--gravity [1-10] --threshold [1-10] --startIndex [Z-Index #] --endAfter [value]} --seg [Seg ID #] --volpkg [volpkgpath]" << std::endl;
    exit(EXIT_FAILURE);
  }

// Option parsing
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

  pcl::console::parse_argument (argc, argv, "--startIndex", startIndex);
  
  pcl::console::parse_argument (argc, argv, "--endAfter", stopOffset);

  pcl::console::parse_argument (argc, argv, "--seg", segID);
  if (segID == "") {
    std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  pcl::console::parse_argument (argc, argv, "--volpkg", volpkgLocation);
  if (volpkgLocation == "") {
    std::cerr << "ERROR: Incorrect/missing volpkg location!" << std::endl;
    exit(EXIT_FAILURE);
  }

// Load volume package
  VolumePkg volpkg(volpkgLocation);
  // To-Do: Check to make sure the Seg ID is actually in the volpkg
  volpkg.setActiveSegmentation(segID);

// Setup
  // Load the activeSegmentation's current cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr masterCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  masterCloud = volpkg.openCloud();
  int chainLength = masterCloud->width;

  // Setup the cloud filter to generate our starting path for segmentation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr immutableCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath (new pcl::PointCloud<pcl::PointXYZRGB>);

  // If no start index is given, our starting path is all of the points already on the largest slice index
  if (startIndex == -1.0) {
    pcl::PointXYZRGB min_p, max_p;
    pcl::getMinMax3D (*masterCloud, min_p, max_p);

    startIndex = floor(max_p.x);
    std::cout << "No starting index given, defaulting to Highest-Z: " << startIndex << std::endl;
  }


// Prepare our clouds
  // Conditions for being a point that is part of the starting path: startIndex <= pt.x < startIndex + 1
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr pathCond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  pathCond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GE, startIndex)));
  pathCond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, startIndex + 1.0)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> pathFilter;
  pathFilter.setCondition(pathCond);
  pathFilter.setInputCloud (masterCloud);  
  // apply filter
  pathFilter.filter (*segPath);

  // starting paths must have the same number of points as the input width to maintain ordering
  if ( segPath->width != chainLength ) {
    std::cerr << std::endl << "ERROR: Starting chain length does not match expected chain length." << std::endl;
    std::cerr << "           Expected: " << chainLength << std::endl;
    std::cerr << "           Actual: " << segPath->width << std::endl;
    std::cerr << "       Consider using a lower starting index value." << std::endl << std::endl;
    exit(EXIT_FAILURE);
  }

  // Conditions for being a point before the starting path: pt.x < startIndex
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr staticCond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  staticCond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, startIndex)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> staticFilter;
  staticFilter.setCondition(staticCond);
  staticFilter.setInputCloud (masterCloud);
  // apply filter
  staticFilter.filter (*immutableCloud);

  
// Run segmentation using path as our starting points
  pcl::PointCloud<pcl::PointXYZRGB> mutableCloud;
  mutableCloud = structureTensorParticleSim(segPath, volpkg, gravity_scale, threshold, stopOffset);

  // Update the master cloud with the points we saved and concat the new points into the space
  *masterCloud = *immutableCloud;
  *masterCloud += mutableCloud;
  
  // Restore ordering information
  masterCloud->width = chainLength;
  masterCloud->height = masterCloud->points.size()/masterCloud->width;
  masterCloud->points.resize (masterCloud->width * masterCloud->height);

// Save point cloud
  volpkg.saveCloud(*masterCloud);
  exit(EXIT_SUCCESS);
}