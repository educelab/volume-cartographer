//#include <opencv2/opencv.hpp>
#include "volumepkgcfg.h"

class VolumePkg {
public:
	VolumePkg(std::string);
private:
	VolumePkgCfg config;
	int getNumberOfSlices();
	//cv::mat getSliceAt(int);
};