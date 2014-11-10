#ifndef _VOLUMEPKG_H_
#define _VOLUMEPKG_H_

#include <opencv2/opencv.hpp>
#include "volumepkgcfg.h"
#include <stdlib.h>

class VolumePkg {
public:
	VolumePkg(std::string);
	int getNumberOfSlices();
	cv::Mat getSliceAtIndex(int);
private:
	VolumePkgCfg config;
	std::string location;
	int getNumberOfSliceCharacters();
};

#endif // _VOLUMEPKG_H_
