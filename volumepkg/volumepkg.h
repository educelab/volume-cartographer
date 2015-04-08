#ifndef _VOLUMEPKG_H_
#define _VOLUMEPKG_H_

#include <stdlib.h>
#include <time.h>

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "volumepkgcfg.h"

class VolumePkg {
public:
	VolumePkg(std::string);
	int getNumberOfSlices();
	std::string getPkgName();
	cv::Mat getSliceAtIndex(int);
	std::string getNormalAtIndex(int);

	// Segmentation functions
	std::vector<std::string> getSegmentations();
	void setActiveSegmentation(std::string);
  std::string newSegmentation();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr openCloud();
	void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>);
	
private:
	VolumePkgCfg config;
	std::string location;
  boost::filesystem::path segdir;
	int getNumberOfSliceCharacters();
  std::string activeSeg = "";
	std::vector<std::string> segmentations;
};

#endif // _VOLUMEPKG_H_
