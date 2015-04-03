#ifndef _VOLUMEPKG_H_
#define _VOLUMEPKG_H_

#include <opencv2/opencv.hpp>
#include "volumepkgcfg.h"
#include <stdlib.h>

class VolumePkg {
public:
	VolumePkg(std::string);
	int getNumberOfSlices();
	std::string getPkgName();
	cv::Mat getSliceAtIndex(int);
	std::string getNormalAtIndex(int);

	// Segmentation functions
	std::vector<int> getSegmentations();
	int setActiveSegmentation(int);
	int newSegmentation();
	pcl::PointCloud<pcl::PointXYZRGB> openCloud();
	int saveCloud(pcl::PointCloud<pcl::PointXYZRGB>);
	
private:
	VolumePkgCfg config;
	std::string location;
	int getNumberOfSliceCharacters();
	int activeSeg;
};

#endif // _VOLUMEPKG_H_
