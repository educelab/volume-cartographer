// main.cpp
// Chao Du Nov 2014

#include "sliceIntersection.h"

#include <stdio.h>

#include "CMesh.h"
#include "CPlyHelper.h"
#include "CPoint.h"

#include "volumepkg.h"

int main( int argc, char *argv[] )
{
	double radius;
	if ( argc < 5 ) {
		std::cout << "Usage: sliceIntersection mesh.ply radius wantBetterTexture(y/n) volPkgPath" << std::endl;
		exit( -1 );
	}

	radius = atof( argv[ 2 ] );

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
	aMaxSliceIndex = ( int )ceil( aPoints.back().x );

	// REVISIT - for debug
	std::cout << "Min slice index: " << aMinSliceIndex << 
				" Max slice index: " << aMaxSliceIndex << std::endl;

	// (1.5) do non-maximum suppression and find the better texture for each vertices
	// REVISIT - improve: to save the time for reading images, read the TIF files at the same time
	//           which would be a heavy burden on memory

	// REVISIT - replace this with VolumePackager
	VolumePkg vpkg = VolumePkg( argv[ 4 ] );
	std::vector< cv::Mat > aImgVol;
	ProcessVolume( vpkg, aImgVol, true, true );

	if ( aIsToFindBetterTexture ) {
		printf( "find better texture\n" );
		FindBetterTexture( aMesh,
							aImgVol,
							radius,//3.0,
							FilterNonMaximumSuppression );
		printf( "writing result\n" );
		ChaoVis::CPlyHelper::WritePlyFile( aFileName + "_mod.ply", aMesh );
	} else {
		printf( "equalized texture\n" );
		FindBetterTexture( aMesh,
							aImgVol,
							radius,//3.0,
							FilterDummy );
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

		sprintf( aImgFileName, aImgPrefix, aCnt );
		cv::imwrite( aImgFileName, *aStackIter );

		// REVISIT - debug, overlay the path to the original image
		std::cout << "Overlaying image " << aCnt << std::endl;
		char aOriginalImgPrefix[128];
		char aOriginalImgFileName[128];
		strcpy( aOriginalImgPrefix, argv[ 5 ] );
		sprintf( aOriginalImgFileName, aOriginalImgPrefix, aCnt );
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
		sprintf( aOutputImgFileName, aOutputImgPrefix, aCnt );
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
