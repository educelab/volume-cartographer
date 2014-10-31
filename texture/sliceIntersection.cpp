// sliceIntersection.cpp
// Chao Du Sept. 2014
// this program intends to project the mesh on each slice and 
// find the texture on it, so we can tell whether the mesh
// grows as the slices indicated.

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "CMesh.h"
#include "CPlyHelper.h"
#include "CPoint.h"


typedef struct pt_tag {
	unsigned char color;
	cv::Vec2i loc;
} pt;


bool OpenSliceImgFile( const std::string &nFileName,
						cv::Mat &nImg );

void GetMeshSliceIntersection( int nSliceIndex,
								const ChaoVis::CMesh &nMesh,
								std::vector< ChaoVis::CPoint2f > &nPath );

void OutputIntersectionOnSlice( const std::string &nFileName,
								const cv::Mat &nImg,
								const std::vector< ChaoVis::CPoint2f > &nPath );

void FindBetterTexture( ChaoVis::CMesh &nMesh,
						const std::vector< cv::Mat > &nImgVol,
						float nRadius,
						int nStartIndex,
						double (*BetterTextureFunc)(double *nData, int nSize) );

double NonMaximumSuppression( double *nData,
								int nSize );

double NonLocalMaximumSuppression( double *nData,
								int nSize );

// estimate intensity of volume at particle
double interpolate_intensity( const cv::Vec3f &point,
								const std::vector< cv::Mat > &nImgVol,
								int nStartIndex )
{
  double dx, dy, dz, int_part;
  // for new data
  dx = modf(point[(0)], &int_part);
  dy = modf(point[(2)], &int_part);
  dz = modf(point[(1)], &int_part);

  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(0);
  x_max = x_min + 1;
  y_min = (int)point(2);
  y_max = y_min + 1;
  z_min = (int)point(1);
  z_max = z_min + 1;

  // safe net
  if ( x_min < 0 || x_max > nImgVol.size() - 1 ||
	  y_min < 0 || y_max > nImgVol[ x_min ].rows - 1 ||
	  z_min < 0 || z_max > nImgVol[ x_min ].cols - 1 ) {

	  return 0;

  }

  double result =
    nImgVol[ x_min ].at< unsigned char/*cv::Vec3b*/ >( y_min, z_min )/*[ 0 ]*/ * (1 - dx) * (1 - dy) * (1 - dz) +
    nImgVol[ x_max ].at< unsigned char/*cv::Vec3b*/ >( y_min, z_min )/*[ 0 ]*/ * dx       * (1 - dy) * (1 - dz) +
    nImgVol[ x_min ].at< unsigned char/*cv::Vec3b*/ >( y_max, z_min )/*[ 0 ]*/ * (1 - dx) * dy       * (1 - dz) +
    nImgVol[ x_min ].at< unsigned char/*cv::Vec3b*/ >( y_min, z_max )/*[ 0 ]*/ * (1 - dx) * (1 - dy) * dz +
    nImgVol[ x_max ].at< unsigned char/*cv::Vec3b*/ >( y_min, z_max )/*[ 0 ]*/ * dx       * (1 - dy) * dz +
    nImgVol[ x_min ].at< unsigned char/*cv::Vec3b*/ >( y_max, z_max )/*[ 0 ]*/ * (1 - dx) * dy       * dz +
    nImgVol[ x_max ].at< unsigned char/*cv::Vec3b*/ >( y_max, z_min )/*[ 0 ]*/ * dx       * dy       * (1 - dz) +
    nImgVol[ x_max ].at< unsigned char/*cv::Vec3b*/ >( y_max, z_max )/*[ 0 ]*/ * dx       * dy       * dz;

  return result;
}

bool CompareXLess( const pcl::PointXYZRGBNormal &nP1,
				   const pcl::PointXYZRGBNormal &nP2 )
{
	return( nP1.x < nP2.x );
}

int main( int argc, char *argv[] )
{
	double radius;
	if ( argc < 5 ) {
		std::cout << "Usage: sliceIntersection mesh.ply radius wantBetterTexture(y/n) startSliceIndex imgNameTemplate" << std::endl;
		exit( -1 );
	}

	radius = atof( argv[ 2 ] );
	const int MIN_SLICE = atoi( argv[ 4 ] );

	bool aIsToFindBetterTexture = ( argv[ 3 ][ 0 ] == 'y' || argv[ 3 ][ 0 ] == 'Y' );
	// (1) read in ply mesh file
	ChaoVis::CMesh aMesh;
	std::string aFileName( argv[ 1 ] );
	ChaoVis::CPlyHelper::ReadPlyFile( aFileName, aMesh );
	std::cout << "Mesh file loaded" << std::endl;
	// REVISIT - for debug
	aMesh.Dump();

	// copy and sort the vertices
	std::vector< pcl::PointXYZRGBNormal > aPoints( aMesh.fPoints.size() );
	std::copy( aMesh.fPoints.begin(), aMesh.fPoints.end(), aPoints.begin() );
	std::sort( aPoints.begin(), aPoints.end(), CompareXLess );

	int aMinSliceIndex, aMaxSliceIndex;

	aMinSliceIndex = ( int )floor( aPoints.begin()->x );


/*	if ( aMinSliceIndex < MIN_SLICE ) {
		aMinSliceIndex = MIN_SLICE;
	}*/
	aMaxSliceIndex = ( int )ceil( aPoints.back().x );

	// REVISIT - for debug
	std::cout << "Min slice index: " << aMinSliceIndex << 
				" Max slice index: " << aMaxSliceIndex << std::endl;

	// (1.5) do non-maximum suppression and find the better texture for each vertices
	// REVISIT - improve: to save the time for reading images, read the TIF files at the same time
	//           which would be a heavy burden on memory
	char aOriginalImgPrefix[128];
	char aOriginalImgFileName[128];

	// REVISIT - replace this with VolumePackager
	strcpy( aOriginalImgPrefix, argv[ 5 ] );
	std::vector< cv::Mat > aImgVol;
	for ( int i = 0; i < aMaxSliceIndex /*+ 2*/; ++i ) {
		sprintf( aOriginalImgFileName, aOriginalImgPrefix, i + MIN_SLICE );
		cv::Mat aImg = cv::imread( aOriginalImgFileName );

#define _LIKE_SCALPEL
#ifdef _LIKE_SCALPEL
		cvtColor( aImg, aImg, CV_BGR2GRAY );
		//printf( "channels: %d, depth: %d\n", aImg.channels(), aImg.depth() );
		GaussianBlur( aImg, aImg, cv::Size( 3, 3 ), 0);
		equalizeHist( aImg, aImg );
#endif // _LIKE_SCALPEL

		aImgVol.push_back( aImg );
	}

	if ( aIsToFindBetterTexture ) {
		printf( "find better texture\n" );
		FindBetterTexture( aMesh,
							aImgVol,
							radius,//3.0,
							MIN_SLICE,
							NonMaximumSuppression );
		printf( "writing result\n" );
		ChaoVis::CPlyHelper::WritePlyFile( aFileName + "_mod.ply", aMesh );
	}

	
	// (2) for each slice, do not read in slice texture file, but directly find
	//     mesh intersection on that slice and draw path
	// REVISIT - the slice on each end of the mesh may or may not have triangle on it
	int aNumSlices = aMaxSliceIndex - aMinSliceIndex + 1;
	std::vector< cv::Mat > aIntrsctColor;
	for ( size_t i = 0; i < aNumSlices; ++i ) {
		aIntrsctColor.push_back( cv::Mat( aImgVol[ 0 ].cols, aImgVol[ 0 ].rows, CV_8UC3 ) );
	}
	std::vector< std::vector< pt > > aIntrsctPos( aNumSlices );

	// iterate through all the edges
	std::set< cv::Vec2i, ChaoVis::EdgeCompareLess >::iterator aIter;
	for ( aIter = aMesh.fEdges.begin(); aIter != aMesh.fEdges.end(); ++aIter ) {

		pcl::PointXYZRGBNormal aV1 = aMesh.fPoints[ ( *aIter )[ 0 ] ];
		pcl::PointXYZRGBNormal aV2 = aMesh.fPoints[ ( *aIter )[ 1 ] ];

		int aStartIndx = ( int )ceil( aV1.x );
		int aEndIndx = ( int )floor( aV2.x );

		// safe net
		if ( aStartIndx < aMinSliceIndex || aEndIndx > aMaxSliceIndex - 1 ) {
			continue;
		}

		// interpolate all the intersection points
		for ( int i = aStartIndx; i <= aEndIndx; ++i ) {
    
			cv::Vec3b aPixel;
			int aRow, aCol;
			if ( fabs( aV2.x - aV1.x ) < 1e-6 ) {
				if ( fabs( aV2.x - i ) < 1e-6 ) {
					// point 1
					aRow = round( aV2.y );
					aCol = round( aV2.z );

					aPixel[ 0 ] = ( unsigned char )aV2.r;
					aPixel[ 1 ] = ( unsigned char )aV2.g;
					aPixel[ 2 ] = ( unsigned char )aV2.b;

					pt aPt;
					aPt.loc = cv::Vec2i( aRow, aCol );
					aPt.color = ( unsigned char )aV2.b;
					aIntrsctPos[ i - aMinSliceIndex ].push_back( aPt );
        
					aIntrsctColor[ i - aMinSliceIndex ].at< cv::Vec3b >( aRow, aCol ) = aPixel;

					// point 2
					aRow = round( aV1.y );
					aCol = round( aV1.z );

					aPixel[ 0 ] = ( unsigned char )aV1.r;
					aPixel[ 1 ] = ( unsigned char )aV1.g;
					aPixel[ 2 ] = ( unsigned char )aV1.b;
        
					aPt.loc = cv::Vec2i( aRow, aCol );
					aPt.color = ( unsigned char )aV1.b;
					aIntrsctPos[ i - aMinSliceIndex ].push_back( aPt );

					aIntrsctColor[ i - aMinSliceIndex ].at< cv::Vec3b >( aRow, aCol ) = aPixel;
				}
				continue;
			}
			double d = ( aV2.x - i ) / ( aV2.x - aV1.x );
    
			aRow = round( d * aV1.y + ( 1.0 - d ) * aV2.y );
			aCol = round( d * aV1.z + ( 1.0 - d ) * aV2.z );

			aPixel[ 0 ] = ( unsigned char )( d * aV1.r + ( 1.0 - d ) * aV2.r );
			aPixel[ 1 ] = ( unsigned char )( d * aV1.g + ( 1.0 - d ) * aV2.g );
			aPixel[ 2 ] = ( unsigned char )( d * aV1.b + ( 1.0 - d ) * aV2.b );

			pt aPt;
			aPt.loc = cv::Vec2i( aRow, aCol );
			aPt.color = ( unsigned char )( d * aV1.b + ( 1.0 - d ) * aV2.b );
			aIntrsctPos[ i - aMinSliceIndex ].push_back( aPt );
    
			aIntrsctColor[ i - aMinSliceIndex ].at< cv::Vec3b >( aRow, aCol ) = aPixel;

		} // for

	} // for

#ifdef _DEBUG
	// output all the path in each slices
	std::vector< cv::Mat >::iterator aStackIter;
	int aCnt = 0;
	char aImgPrefix[128];
	char aImgFileName[128];
	strcpy( aImgPrefix, "/home/chaodu/Research/Scroll/TestData/Sphere_Papyri-Sample-2005/texture/XY%04d.png" );
	for ( aStackIter = aIntrsctColor.begin(); aStackIter != aIntrsctColor.end(); ++aStackIter, ++aCnt ) {

		sprintf( aImgFileName, aImgPrefix, aCnt + MIN_SLICE );
		cv::imwrite( aImgFileName, *aStackIter );

		// REVISIT - debug, overlay the path to the original image
		std::cout << "Overlaying image " << aCnt << std::endl;
		char aOriginalImgPrefix[128];
		char aOriginalImgFileName[128];
		strcpy( aOriginalImgPrefix, argv[ 5 ] );
		sprintf( aOriginalImgFileName, aOriginalImgPrefix, aCnt + MIN_SLICE );
		// REVISIT - since we have aImgVol, don't need to read image again
		cv::Mat aOriginalImg;
		aImgVol.at( aCnt ).copyTo( aOriginalImg );
		for ( std::vector< /*cv::Vec2i*/pt >::iterator aPathPtIter = aIntrsctPos[ aCnt ].begin(); 
				aPathPtIter != aIntrsctPos[ aCnt ].end(); 
				++aPathPtIter ) {
			
			aOriginalImg.at< cv::Vec3b >( ( *aPathPtIter ).loc[ 1 ], ( *aPathPtIter ).loc[ 0 ] ) = cv::Vec3b( 0, 0, 255 );

		}

		char aOutputImgPrefix[128];
		char aOutputImgFileName[128];
		strcpy( aOutputImgPrefix, "/home/chaodu/Research/Scroll/TestData/Sphere_Papyri-Sample-2005/texture/XY%04d.png" );
		sprintf( aOutputImgFileName, aOutputImgPrefix, aCnt + MIN_SLICE );
		cv::imwrite( aOutputImgFileName, aOriginalImg );

		// for debug
//		system( "ffmpeg -framerate 25 -start_number 3 -i ../../scroll_sample_data/texture/scroll2_239um_oversize_offset_take2_%03d.tif -c:v libx264 ../../scroll_sample_data/texture/out.mp4" );
//		system( "ffmpeg -framerate 25 -start_number 0 -i \"/home/chaodu/Desktop/Franklin Samples/Franklin_Scan1_10um_2240x600+0+525.volumepkg/texture/%04d.tif\" -c:v libx264 \"/home/chaodu/Desktop/Franklin Samples/Franklin_Scan1_10um_2240x600+0+525.volumepkg/texture/out.mp4\"" );

	}
#endif // _DEBUG



	// create mp4 movie
	// note: with -pix_fmt option the generated file can have the largest compatibility, however,
	//       we have to make the dimensions to be even numbers
	// for compatible output
//	system( "ffmpeg -framerate 25 -start_number 3 -i ../../scroll_sample_data/texture/scroll2_239um_oversize_offset_take2_%03d.tif -c:v libx264 -pix_fmt yuv420p -vf \"scale=trunc(iw/2)*2:trunc(ih/2)*2\" ../../scroll_sample_data/texture/out.mp4" );

	return 0;
}

// open slice image file and store as rgb8_image_t
bool OpenSliceImgFile( const std::string &nFileName,
						cv::Mat &nImg )
{
	return false;
}

// get the intersection of the mesh and the particular slice
void GetMeshSliceIntersection( int nSliceIndex,
								const ChaoVis::CMesh &nMesh,
								std::vector< ChaoVis::CPoint2f > &nPath )
{
}

// draw intersection path on the particular slice
void OutputIntersectionOnSlice( const std::string &nFileName,
								const cv::Mat &nImg,
								const std::vector< ChaoVis::CPoint2f > &nPath )
{
}

void FindBetterTexture( ChaoVis::CMesh &nMesh,
						const std::vector< cv::Mat > &nImgVol,
						float nRadius,
						int nStartIndex,
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
													nImgVol,
													nStartIndex );

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
double NonMaximumSuppression( double *nData,
								int nSize )
{
	double aResult = 0.0;

	// find the maximum and update the texture
	for ( int i = 0; i < nSize; ++i ) {
		if ( nData[ i ] > aResult ) {
			aResult = nData[ i ];
		}
	}

	return aResult;
}

// function pointer for selecting the best texture
double NonLocalMaximumSuppression( double *nData,
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
				index = nData/*aSamples*/[ aCenterIndex + i ] > nData/*aSamples*/[ aCenterIndex - i ] ?
						aCenterIndex + i : aCenterIndex - i;
			} else if ( aIsLocalMax[ aCenterIndex + i ] ) {
				index = aCenterIndex + i;
			} else if ( aIsLocalMax[ aCenterIndex - i ] ) {
				index = aCenterIndex - i;
			}
			unsigned char c = nData/*aSamples*/[ index ];

//			if ( c > aIter->b ) { // only update when we see something brighter
			if ( c > aResult ) {
				uint32_t color =
							c |
							c << 8 |
							c << 16;
//				aIter->rgb = *reinterpret_cast<float*>(&color);
				aResult = c;
				// REVISIT - update particle's location
//				cv::Vec3f aNewPos = aFarthest2 + index * aNormalVec * SAMPLE_RATE;
//				aIter->x = aNewPos[ 0 ];
//				aIter->y = aNewPos[ 1 ];
//				aIter->z = aNewPos[ 2 ];
                
				// break: nearest local maxima; don't break: global maxima
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
