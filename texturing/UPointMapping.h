// UPointMapping.h
// Chao Du 2015 Jan
#ifndef _UPOINTMAPPING_H_
#define _UPOINTMAPPING_H_

#include <opencv2/opencv.hpp>

void CalcHomographyFromPoints( const std::vector< cv::Vec3d > &nPtSrc,
                               const std::vector< cv::Vec3d > &nPtTgt,
                               cv::Mat &nH );

void CalcMappedPoint( const std::vector< cv::Vec3d > &nPtSrc,
                      std::vector< cv::Vec3d > &nPtTgt,
                      const cv::Mat &nH );

#endif // _UPOINTMAPPING_H_
