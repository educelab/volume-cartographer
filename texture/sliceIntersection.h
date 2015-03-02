// sliceIntersection.h
// Chao Du Nov 2014
#ifndef _SLICEINTERSECTION_H_
#define _SLICEINTERSECTION_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "CMesh.h"

#include "volumepkg.h"

#define NUM_BITS_PER_BYTE 8
//#define _DEBUG

typedef struct pt_tag {
	unsigned char color;
	cv::Vec2i loc;
} pt;

// estimate intensity of volume at particle
inline
double interpolate_intensity( const cv::Vec3f					&point,
								const std::vector< cv::Mat >	&nImgVol );

bool CompareXLess( const pcl::PointXYZRGBNormal &nP1,
				   const pcl::PointXYZRGBNormal &nP2 );

void FindBetterTexture( ChaoVis::CMesh					&nMesh,
						const std::vector< cv::Mat >	&nImgVol,
						float							nRadius,
						double							(*BetterTextureFunc)(double *nData, int nSize) );

void FindBetterTextureMedianFilter( ChaoVis::CMesh &nMesh,
						const std::vector< cv::Mat > &nImgVol,
						float nMajorAxislen,
                        float nMinorAxisLen,
						//int nStartIndex,
						double (*BetterTextureFunc)(double *nData, int nSize) );

double FilterNonMaximumSuppression( double	*nData,
									int		nSize );

double FilterNonLocalMaximumSuppression( double *nData,
										int		nSize );

double FilterDummy ( double	*nData,
					int		nSize );

void ProcessVolume( /*const*/ VolumePkg		&nVpkg, 
					std::vector< cv::Mat >	&nImgVol,
					bool					nNeedEqualize = false,
					bool					nNeedNormalize = true );

void SamplingWithinEllipse( double nA,
                            double nB,
                            double nDensity,
                            const cv::Vec3f &nCenter,
                            const cv::Vec3f &nMajorAxisDir,
                            const std::vector< cv::Mat > &nImgVol,
                            double *nData,
                            int *nSize );

double FilterMedianAverage( double *nData,
                            int    nSize );

double FilterMin( double *nData,
                  int    nSize );

double FilterMedian( double *nData,
                     int    nSize );

double FilterMean( double *nData,
                   int    nSize );

#endif // _SLICEINTERSECTION_H_
