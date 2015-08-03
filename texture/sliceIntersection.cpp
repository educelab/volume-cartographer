// sliceIntersection.cpp
// Chao Du Sept. 2014
// this program intends to project the mesh on each slice and 
// find the texture on it, so we can tell whether the mesh
// grows as the slices indicated.

#include "sliceIntersection.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include <string>

#include "CMesh.h"
#include "CPlyHelper.h"
#include "CPoint.h"

#include "volumepkg.h"

#ifdef _NEED_DENOISE_
#include "denoiose_tvl1.h"
#endif // _NEED_DENOISE_

//#define gNumHistBin	65536//256 // REVISIT - gNumHistBin = 1 << 8 * sizeof( typename )
unsigned int gNumHistBin;


// estimate intensity of volume at particle
double interpolate_intensity( const cv::Vec3f					&point,
															const std::vector< cv::Mat >	&nImgVol )
{
  double dx, dy, dz, int_part;
  // for new data
  dx = modf(point[(0)], &int_part);
  dy = modf(point[(1)], &int_part);
  dz = modf(point[(2)], &int_part);

  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(0);
  x_max = x_min + 1;
  y_min = (int)point(1);
  y_max = y_min + 1;
  z_min = (int)point(2);
  z_max = z_min + 1;

  // safe net
	if ( z_min < 0 || z_max > nImgVol.size() - 1 ||
	     x_min < 0 || x_max > nImgVol[ z_min ].cols - 1 ||
	     y_min < 0 || y_max > nImgVol[ z_min ].rows - 1 ) {

	  return 0;

  }

  double result =
    nImgVol[ z_min ].at< unsigned char >( y_min, x_min ) * (1 - dx) * (1 - dy) * (1 - dz) +
    nImgVol[ z_min ].at< unsigned char >( y_min, x_max ) * dx       * (1 - dy) * (1 - dz) +
    nImgVol[ z_min ].at< unsigned char >( y_max, x_min ) * (1 - dx) * dy       * (1 - dz) +
    nImgVol[ z_max ].at< unsigned char >( y_min, x_min ) * (1 - dx) * (1 - dy) * dz +
    nImgVol[ z_max ].at< unsigned char >( y_min, x_max ) * dx       * (1 - dy) * dz +
    nImgVol[ z_max ].at< unsigned char >( y_max, x_min ) * (1 - dx) * dy       * dz +
    nImgVol[ z_min ].at< unsigned char >( y_max, x_max ) * dx       * dy       * (1 - dz) +
    nImgVol[ z_max ].at< unsigned char >( y_max, x_max ) * dx       * dy       * dz;

  return result;
}

bool CompareXLess( const pcl::PointXYZRGBNormal &nP1,
				   const pcl::PointXYZRGBNormal &nP2 )
{
	return( nP1.z < nP2.z );
}

void FindBetterTexture( ChaoVis::CMesh &nMesh,
						const std::vector< cv::Mat > &nImgVol,
						float nRadius,
						//int nStartIndex,
						double (*BetterTextureFunc)(double *nData, int nSize) )
{
	// find the NEAREST local maximum, with regular sampling
	// for example:
	// +--+--+--O--+--+--+
	/* /\  /\/\__  /\    / */
	/*   \/      \/  \__/  */
	// however, note that this is inaccurate, because we use trilinear interpolation for the color
	// at location not at the vertices, the intensity curve between vertices is linear
	const double SAMPLE_RATE = 0.1;
	const int TOTAL_SAMPLING_NUM = 2 * nRadius / SAMPLE_RATE + 1; // 1 more at the center
	double *aSamples = new double[ TOTAL_SAMPLING_NUM + 2 ]; // 1 more at each end

	// do non-maximum suppression around a point to find the best texture value for it 
	std::vector< pcl::PointXYZRGBNormal >::iterator aIter;
	for ( aIter = nMesh.fPoints.begin(); aIter != nMesh.fPoints.end(); ++aIter ) {

		for ( int i = 0; i < TOTAL_SAMPLING_NUM; ++i ) {
			aSamples[ i ] = 0.0;
		}
		aSamples[ TOTAL_SAMPLING_NUM ] = 0.0;
		aSamples[ TOTAL_SAMPLING_NUM + 1 ] = 0.0;

		cv::Vec3f aNormalVec( aIter->normal[ 0 ], aIter->normal[ 1 ], aIter->normal[ 2 ] );
		cv::normalize( aNormalVec, aNormalVec );
		cv::Vec3f aPos( aIter->x, aIter->y, aIter->z );

		// in RxRxR sphere, find the local maximum along the normal direction
		// or, do experiment, find local maximum along +/- normal direction, or the entire sphere
		cv::Vec3f aFarthest1 = aPos + nRadius * ( 1 + SAMPLE_RATE ) * aNormalVec;
		cv::Vec3f aFarthest2 = aPos - nRadius * ( 1 + SAMPLE_RATE ) * aNormalVec;

		// sample the voxel surrounding the point and find the local maxima
		for ( int i = 0; i < TOTAL_SAMPLING_NUM + 2; ++i ) {

			cv::Vec3f aP = aFarthest2 + i * aNormalVec * SAMPLE_RATE;

			aSamples[ i ] = interpolate_intensity( aP,
													nImgVol );

		}

		unsigned char c = ( unsigned char )( BetterTextureFunc( aSamples, TOTAL_SAMPLING_NUM + 2 ) );
		uint32_t color =
					c |
					c << 8 |
					c << 16;
		aIter->rgb = *reinterpret_cast<float*>(&color);
	}

	// clean up
	if ( aSamples != NULL ) {
		delete []aSamples;
		aSamples = NULL;
	}
}

// function pointer for selecting the best texture
double FilterNonMaximumSuppression( double	*nData,
									int		nSize )
{
	double aResult = 0.0;

	// find the maximum and update the texture
	for ( int i = 0; i < nSize; ++i ) {
		if ( nData[ i ] > aResult ) {
			aResult = nData[ i ];

			// REVISIT - TODO update the vertex position

		}
	}

	return aResult;
}

// function pointer for selecting the best texture
double FilterNonLocalMaximumSuppression( double *nData,
								int nSize )
{
	int TOTAL_SAMPLING_NUM = nSize - 2;
	double aResult = 0.0;
	bool *aIsLocalMax = new bool[ TOTAL_SAMPLING_NUM ];

	for ( int i = 0; i < TOTAL_SAMPLING_NUM; ++i ) {
		aIsLocalMax[ i ] = false;
	}

	// find the nearest maximum and update the texture
	int aCenterIndex = ( TOTAL_SAMPLING_NUM - 1 ) / 2;
	for ( int i = 0; i < ( TOTAL_SAMPLING_NUM + 1 ) / 2; ++i ) {
		if ( aIsLocalMax[ aCenterIndex + i ] || aIsLocalMax[ aCenterIndex - i ] ) {
			int index;
			if ( aIsLocalMax[ aCenterIndex + i ] && aIsLocalMax[ aCenterIndex - i ] ) {
				index = nData[ aCenterIndex + i ] > nData[ aCenterIndex - i ] ?
						aCenterIndex + i : aCenterIndex - i;
			} else if ( aIsLocalMax[ aCenterIndex + i ] ) {
				index = aCenterIndex + i;
			} else if ( aIsLocalMax[ aCenterIndex - i ] ) {
				index = aCenterIndex - i;
			}
			unsigned char c = nData[ index ];

			if ( c > aResult ) {
				uint32_t color =
							c |
							c << 8 |
							c << 16;
				aResult = c;
				// REVISIT - update particle's location
//				cv::Vec3f aNewPos = aFarthest2 + index * aNormalVec * SAMPLE_RATE;
//				aIter->x = aNewPos[ 0 ];
//				aIter->y = aNewPos[ 1 ];
//				aIter->z = aNewPos[ 2 ];
                
				//break;
			}
		}
	}

	// clean up
	if ( aIsLocalMax != NULL ) {
		delete []aIsLocalMax;
		aIsLocalMax = NULL;
	}

	return aResult;
}

double FilterDummy( double *nData,
					int nSize )
{
	return nData[ nSize / 2 ];
}

// global histogram equalization
template< typename HistT, typename ImgT >
void GetGlobalHistogram( const std::vector< cv::Mat > &nImgVol,
						HistT *nHist )
{
	HistT *aNewHist = new HistT[ gNumHistBin ];
	memset( aNewHist, 0, sizeof( HistT ) * gNumHistBin );

	for ( std::vector< cv::Mat >::const_iterator aIter = nImgVol.begin(); aIter != nImgVol.end(); ++aIter ) {
		for ( int aRow = 0; aRow < aIter->rows; ++aRow ) {
			for ( int aCol = 0; aCol < aIter->cols; ++aCol ) {
				//int index = aIter->at< cv::Vec3b >( aRow, aCol )[ 0 ];
				int index = aIter->at< ImgT >( aRow, aCol );
    
				assert( index >= 0 && index < gNumHistBin );
    
				aNewHist[ index ]++;
			}
		}
	}

	// first average to avoid overflow
	// REVISIT - do division first will lose some accuracy
	for ( int i = 0; i < gNumHistBin; ++i ) {
		aNewHist[ i ] /= nImgVol.size();
	}

	// do accumulation
	for ( int i = 1; i < gNumHistBin; ++i ) {
		aNewHist[ i ] = aNewHist[ i - 1 ] + aNewHist[ i ];
	}

	// normalize the new histogram to [ 0, 255 ]
	// REVISIT - aFactor == 0?
	unsigned int aFactor = aNewHist[ gNumHistBin - 1] - aNewHist[ 0 ];
	for ( int i = 0; i < gNumHistBin; ++i ) {
		// REVISIT - casting to integer still has the danger that it's out of range of 8-bit char
		nHist[ i ] = static_cast< HistT >( ( aNewHist[ i ] - aNewHist[ 0 ] ) * ( gNumHistBin - 1.0 ) / aFactor );
	}

	delete []aNewHist;
}

// do image histogram equalization
// now only works on grayscale images, cv::Mat of cv::Vec3b
template< typename HistT, typename ImgT >
void DoHistogramEqualization( HistT *nHist,
								const cv::Mat &nSrc,
								cv::Mat &nDest )
{
	// remap the image values
	for ( int aRow = 0; aRow < nSrc.rows; ++aRow ) {
		for ( int aCol = 0; aCol < nSrc.cols; ++aCol ) {
			ImgT aNewColor = nHist[ nSrc.at< ImgT >( aRow, aCol ) ];
			nDest.at< ImgT >( aRow, aCol ) = aNewColor;
		}
	}
}

template< typename ImgT >
void DoNormalization( std::vector< cv::Mat > &nImgVol )
{
	ImgT aMin, aMax;
	aMin = gNumHistBin;
	aMax = 0;
	// linearly map [min, max] to [0, gNumHistBin]
	for ( std::vector< cv::Mat >::iterator aIter = nImgVol.begin(); aIter != nImgVol.end(); ++aIter ) {
		for ( int aRow = 0; aRow < ( *aIter ).rows; ++aRow ) {
			for ( int aCol = 0; aCol < ( *aIter ).cols; ++aCol ) {
				ImgT aValue = aIter->at< ImgT >( aRow, aCol );
				if ( aValue > aMax ) {
					aMax = aValue;
				}
				if ( aValue < aMin ) {
					aMin = aValue;
				}
			}
		}
	}

	for ( std::vector< cv::Mat >::iterator aIter = nImgVol.begin(); aIter != nImgVol.end(); ++aIter ) {
		for ( int aRow = 0; aRow < ( *aIter ).rows; ++aRow ) {
			for ( int aCol = 0; aCol < ( *aIter ).cols; ++aCol ) {
				ImgT aValue = aIter->at< ImgT >( aRow, aCol );
				aIter->at< ImgT >( aRow, aCol ) = ( double )( aValue - aMin ) / ( aMax - aMin ) * ( gNumHistBin - 0 - 1.0 );
			}
		}
	}
}

template< typename HistT, typename ImgT >
void DoGlobalHistogramEqualization( std::vector< cv::Mat > &nImgVol )
{
	HistT *aGlobalHist = new HistT[ gNumHistBin ];
	memset( aGlobalHist, 0, sizeof( HistT ) * gNumHistBin );

#ifdef _NEED_DENOISE_
	// denoise
	for ( vector< Mat >::iterator aIter = nImgVol.begin(); aIter != nImgVol.end(); ++aIter ) {
		vector< Mat > aMatVec;
		aMatVec.push_back( *aIter );
		denoise_TVL1( aMatVec,//aImgVol,
						*aIter, 0.5 );
	}
#endif // _NEED_DENOISE_

	// get histogram and mapping
	GetGlobalHistogram< HistT, ImgT >( nImgVol,
										aGlobalHist );

#ifdef _DEBUG
	// plot the new histogram
	for ( int i = 0; i < gNumHistBin; ++i ) {
		printf( "%03d %d", i, aGlobalHist[ i ] );
/*		for ( int j = 0; j < aGlobalHist[ i ]; ++j ) {
			printf( "*" );
		}
		*/
		printf( "\n" );
	}
#endif // _DEBUG

	int aCnt = 0;
	cv::Mat aOutImg( nImgVol[ 0 ].rows, nImgVol[ 0 ].cols, nImgVol[ 0 ].type() );
	for ( std::vector< cv::Mat >::iterator aIter = nImgVol.begin(); aIter != nImgVol.end(); ++aIter, ++aCnt ) {
		
		aIter->copyTo( aOutImg );

		DoHistogramEqualization< HistT, ImgT >( aGlobalHist,
												*aIter,
												aOutImg );

		aOutImg.copyTo( *aIter );
#ifdef _DEBUG
		char filename[ 50 ];
		sprintf( filename, "%03d.png", aCnt );
		cv::imwrite( filename, aOutImg );
#endif // _DEBUG
	}

	// clean up
	delete []aGlobalHist;
}

void ConvertData16Uto8U( std::vector< cv::Mat > &nImgVol ) {
	for ( size_t i = 0; i < nImgVol.size(); ++i ) {
		std::cout << "\rDownsampling slices to 8bpc: " << i+1 << "/" << nImgVol.size() << std::flush;
        cv::Mat aOriginImg( nImgVol[ i ].rows, nImgVol[ i ].cols, CV_16UC1 );
		nImgVol[ i ].copyTo( aOriginImg );
		nImgVol[ i ].convertTo( nImgVol[ i ], CV_8U );

		for ( int row = 0; row < aOriginImg.rows; ++row ) {
			for ( int col = 0; col < aOriginImg.cols; ++col ) {
				nImgVol[ i ].at< unsigned char >( row, col ) = aOriginImg.at< unsigned short >( row, col ) / 256.0;
			}
		}
	}
    std::cout << std::endl;
}

void ProcessVolume( /*const*/ VolumePkg &nVpkg, 
					std::vector< cv::Mat > &nImgVol,
					bool nNeedEqualize,
					bool nNeedNormalize )
{
    int aNumSlices = nVpkg.getNumberOfSlices();
	for ( int i = 0; i < aNumSlices; ++i ) {
        std::cout << "\rLoading slice: " << i+1 << "/" << aNumSlices << std::flush;
		nImgVol.push_back( nVpkg.getSliceData( i ).clone() );
	}
    std::cout << std::endl;
	gNumHistBin = 1 << ( NUM_BITS_PER_BYTE * sizeof( unsigned short ) );
#ifdef _DEBUG
	printf( "# of bin: %d\n", gNumHistBin );
#endif // _DEBUG
	// optional histogram equalization
	// REVISIT - we have multiple choices: histogram equalization, normalization
	//           this function is always called because 8-bit images are upgraded and stored in 16-bit images,
	//           naively map them back to 8-bit images will generate undesired dark effect
	//           without the second parameter, DoGlobalHistogramEqualization only find the range
	//           of the color intensity and do a equalization
	if ( nNeedEqualize ) {
		DoGlobalHistogramEqualization< unsigned int, unsigned short >( nImgVol );
	}
	if ( nNeedNormalize ) {
		DoNormalization< unsigned short >( nImgVol );
	}
	ConvertData16Uto8U( nImgVol );
}
