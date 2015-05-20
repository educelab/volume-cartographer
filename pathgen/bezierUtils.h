// bezierUtils.h
// Chao Du 2014 Nov
#ifndef _BEZIERUTILS_H_
#define _BEZIERUTILS_H_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "volumepkg.h"


#define CURVE_SAMPLE_RATE 1//0.5


typedef struct pt_tuple_tag {
	cv::Vec2f start;
	cv::Vec2f middle;
	cv::Vec2f end;
} pt_tuple;

float GetPt( int n1, 
			int n2, 
			float percent );

void DrawBezier( cv::Vec2f &start, 
				cv::Vec2f &middle, 
				cv::Vec2f &end,
				cv::Mat &nImg );

bool SavePath( VolumePkg *vpkg,
				const std::vector< pt_tuple > &nPath,
				int nPathOnSliceIndex );

#endif // _BEZIERUTIS_H_
