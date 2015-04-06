#ifndef _VOLUMEPKG_H_
#define _VOLUMEPKG_H_

#include <stdlib.h>
#include <time.h>

#include "boost/filesystem.hpp"
#include <opencv2/opencv.hpp>
#include "volumepkgcfg.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class VolumePkg {
public:
	VolumePkg(std::string);
	int getNumberOfSlices();
	std::string getPkgName();
	cv::Mat getSliceAtIndex(int);
	std::string getNormalAtIndex(int);

	// Segmentation functions
	std::vector<std::string> getSegmentations();
	int setActiveSegmentation(int);
	int newSegmentation();
	pcl::PointCloud<pcl::PointXYZRGB> openCloud();
	int saveCloud(pcl::PointCloud<pcl::PointXYZRGB>);
	
private:
	VolumePkgCfg config;
	std::string location;
    boost::filesystem::path segdir;
	int getNumberOfSliceCharacters();
	int activeSeg = -1;
	std::vector<std::string> segmentations;
};

#endif // _VOLUMEPKG_H_
