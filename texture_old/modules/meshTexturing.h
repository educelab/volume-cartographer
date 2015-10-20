// meshTexturing.h
// Chao Du 2015 Mar

// mesh texturing module
#ifndef _MESHTEXTURING_H_
#define _MESHTEXTURING_H_

#include "../CMesh.h"
#include "../CPlyHelper.h"
#include "../CPoint.h"

#include "volumepkg.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#define NUM_BITS_PER_BYTE 8
//#define _DEBUG

enum EFilterOption {
    FilterOptionIntersection = 0,
    FilterOptionMax,
    FilterOptionMin,
    FilterOptionMedianAverage,
    FilterOptionMedian,
    FilterOptionMean
};

enum EDirectionOption {
    DirectionOptionBoth = 0,
    DirectionOptionPositive,
    DirectionOptionNegative
};

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
                        int                             nSamplingDir,
						double							(*BetterTextureFunc)(double *nData, int nSize) );

void FindBetterTextureMedianFilter( ChaoVis::CMesh &nMesh,
						const std::vector< cv::Mat > &nImgVol,
						float nMajorAxislen,
                        float nMinorAxisLen,
						//int nStartIndex,
                        int   nSamplingDir,
						double (*BetterTextureFunc)(double *nData, int nSize) );

double FilterNonMaximumSuppression( double	*nData,
									int		nSize );

double FilterNonLocalMaximumSuppression( double *nData,
										int		nSize );

double FilterDummy ( double	*nData,
					int		nSize );

void ProcessVolume( /*const*/ VolumePkg		&nVpkg,
                    const ChaoVis::CMesh    &nMesh,
					std::vector< cv::Mat >	&nImgVol,
                    const double            &nRadius,
					bool					nNeedEqualize = false,
					bool					nNeedNormalize = true );

void SamplingWithinEllipse( double nA,
                            double nB,
                            double nDensity,
                            const cv::Vec3f &nCenter,
                            const cv::Vec3f &nMajorAxisDir,
                            const std::vector< cv::Mat > &nImgVol,
                            int nSamplingDir,
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

void meshTexturing( ChaoVis::CMesh      &nMesh, // mesh
                    /*const*/ VolumePkg     &nVpkg, // volume package
                    double              nR1 = 3.0, // radius 1
                    double              nR2 = 1.0, // radius 2
                    EFilterOption       nFilter = FilterOptionIntersection, // filter option
                    EDirectionOption    nDir = DirectionOptionBoth ); // direction option

#endif // _MESHTEXTURING_H_
