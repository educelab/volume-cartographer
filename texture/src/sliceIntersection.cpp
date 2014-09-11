// sliceIntersection.cpp
// Chao Du Sept. 2014
// this program intends to project the mesh on each slice and 
// find the texture on it, so we can tell whether the mesh
// grows as the slices indicated.

#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include <boost/gil/image.hpp>
#include <boost/gil/typedefs.hpp>
#include <boost/gil/extension/io/png_io.hpp>

#include <opencv2/opencv.hpp>

#include "CMesh.h"
#include "CPlyHelper.h"



bool CompareXLess( const pcl::PointXYZRGBNormal &nP1,
				   const pcl::PointXYZRGBNormal &nP2 )
{
	return( nP1.x < nP2.x );
}

int main( int argc, char *argv[] )
{
	if ( argc < 2 ) {
		std::cout << "Usage: sliceIntersection mesh.ply" << std::endl;
		exit( -1 );
	}
	// (1) read in ply mesh file
	ChaoVis::CMesh aMesh;
	std::string aFileName( argv[1] );
	ChaoVis::CPlyHelper::ReadPlyFile( aFileName, aMesh );
	// REVISIT - for debug
	aMesh.Dump();

	// copy and sort the vertices
	std::vector< pcl::PointXYZRGBNormal > aPoints( aMesh.fPoints.size() );
	std::copy( aMesh.fPoints.begin(), aMesh.fPoints.end(), aPoints.begin() );
	std::sort( aPoints.begin(), aPoints.end(), CompareXLess );
	int aMinSliceIndex, aMaxSliceIndex;
	aMinSliceIndex = ( int )floor( aPoints.begin()->x );
	aMaxSliceIndex = ( int )ceil( aPoints.back().x );
	// REVISIT - for debug
	std::cout << "Min slice index: " << aMinSliceIndex << 
				" Max slice index: " << aMaxSliceIndex << std::endl;
	
	// (2) for each slice, don not read in slice texture file, directly find
	//     mesh intersection on that slice and draw path
	// REVISIT - the slice on each end of the mesh may or may not have triangle on it
	int aNumSlices = aMaxSliceIndex - aMinSliceIndex + 1;
	// REVISIT - image size
	std::vector< boost::gil::rgb8_image_t > aIntrsctColor( aNumSlices,
														boost::gil::rgb8_image_t(750, 750) );
	std::vector< std::vector< cv::Vec2i > > aIntrsctPos( aNumSlices );

	// iterate through all the edges
	std::set< cv::Vec2i, ChaoVis::EdgeCompareLess >::iterator aIter;
	for ( aIter = aMesh.fEdges.begin(); aIter != aMesh.fEdges.end(); ++aIter ) {

		pcl::PointXYZRGBNormal aV1 = aMesh.fPoints[ (*aIter)[ 0 ] ];
		pcl::PointXYZRGBNormal aV2 = aMesh.fPoints[ (*aIter)[ 1 ] ];

		int aStartIndx = ( int )ceil( aV1.x );
		int aEndIndx = ( int )floor( aV2.x );

		// interpolate all the intersection points
		for ( int i = aStartIndx; i <= aEndIndx; ++i ) {
    
			boost::gil::rgb8_pixel_t aPixel;
			int aRow, aCol;
			int r, g, b;
			if ( fabs( aV2.x - aV1.x ) < 1e-6 ) {
				continue;
			}
			double d = ( aV2.x - i ) / ( aV2.x - aV1.x );
    
			aRow = round( d * aV1.y + ( 1.0 - d ) * aV2.y );
			aCol = round( d * aV1.z + ( 1.0 - d ) * aV2.z );

			aIntrsctPos[ i - aMinSliceIndex ].push_back( cv::Vec2i( aRow, aCol ) );
    
			r = d * aV1.r + ( 1.0 - d ) * aV2.r;
			g = d * aV1.g + ( 1.0 - d ) * aV2.g;
			b = d * aV1.b + ( 1.0 - d ) * aV2.b;
    
			aPixel[ 0 ] = ( unsigned char )r;
			aPixel[ 1 ] = ( unsigned char )g;
			aPixel[ 2 ] = ( unsigned char )b;
    
			boost::gil::view( aIntrsctColor[ i - aMinSliceIndex ] )( aRow, aCol ) = aPixel;

		} // for

	} // for

	// output all the path in each slices
	
	std::vector< boost::gil::rgb8_image_t >::iterator aStackIter;
	int aCnt = 0;
	char aImgPrefix[128];
	char aImgFileName[128];
	strcpy( aImgPrefix, "../../scroll_sample_data/texture/texture_%04d.png" );
	for ( aStackIter = aIntrsctColor.begin(); aStackIter != aIntrsctColor.end(); ++aStackIter, ++aCnt ) {

		sprintf( aImgFileName, aImgPrefix, aCnt );
		boost::gil::png_write_view( aImgFileName, boost::gil::view( *aStackIter ) );

		// REVISIT - debug, overlay the path to the original image
		std::cout << "Overlaying image " << aCnt << std::endl;
		char aOriginalImgPrefix[128];
		char aOriginalImgFileName[128];
		strcpy( aOriginalImgPrefix, "../../scroll_sample_data/slices/scroll2_239um_oversize_offset_take2_%03d_rec%04d.tif" );
		sprintf( aOriginalImgFileName, aOriginalImgPrefix, aCnt + 3, 6373 - aCnt );
		cv::Mat aOriginalImg = cv::imread( aOriginalImgFileName );
		for ( std::vector< cv::Vec2i >::iterator aPathPtIter = aIntrsctPos[ aCnt ].begin(); 
				aPathPtIter != aIntrsctPos[ aCnt ].end(); 
				++aPathPtIter ) {
			
			aOriginalImg.at< cv::Vec3b >( ( *aPathPtIter )[ 1 ], ( *aPathPtIter )[ 0 ] ) = cv::Vec3b( 0, 0, 255 );

		}
		cv::imwrite( strcat( aOriginalImgFileName, "_mod.tif" ), aOriginalImg );
	}

	return 0;
}
