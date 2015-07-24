// sliceProjection.cpp
// Chao Du 2014 Nov
// Project the mesh onto each slices in order to check the quality of segmentation
#include <stdio.h>

#include "CMesh.h"
#include "CPlyHelper.h"
#include "CPoint.h"

#include "volumepkg.h"
#include "sliceIntersection.h"


int main( int argc, char *argv[] )
{
    printf( "Running tool: sliceProjection\n" );
    std::cout << std::endl;
    if ( argc < 4 ) {
        printf( "Usage: sliceProjection mesh.ply volPkgPath outputPath\n" );
        exit( -1 );
    }

    // (1) read in ply mesh file
    ChaoVis::CMesh aMesh;
    std::string aFileName( argv[ 1 ] );
    // REVISIT - IMPORTANT!!! if the mesh ply file contains "nan", the mesh won't be read in correctly
    //           this typically happens for the normals of vertices
    std::cout << "Loading mesh..." << std::endl;
    ChaoVis::CPlyHelper::ReadPlyFile( aFileName, aMesh );
    // REVISIT - for debug
    aMesh.Dump();
    std::cout << "Mesh file loaded" << std::endl;

    // copy and sort the vertices
    std::vector< pcl::PointXYZRGBNormal > aPoints( aMesh.fPoints.size() );
    std::copy( aMesh.fPoints.begin(), aMesh.fPoints.end(), aPoints.begin() );
    std::sort( aPoints.begin(), aPoints.end(), CompareXLess );

    int aMinSliceIndex, aMaxSliceIndex;

    aMinSliceIndex = ( int )floor( aPoints.begin()->z );
    if (aMinSliceIndex == -1 ){
        aMinSliceIndex = 0;
    }
    aMaxSliceIndex = ( int )ceil( aPoints.back().z );

    // REVISIT - for debug
    std::cout << std::endl;
    std::cout << "Min slice index: " << aMinSliceIndex << 
                " Max slice index: " << aMaxSliceIndex << std::endl;

    // (2) load in volume package
    VolumePkg aVpkg = VolumePkg( argv[ 2 ] );
    std::vector< cv::Mat > aImgVol;
    ProcessVolume( aVpkg, aImgVol, false, false );
    std::cout << std::endl;

    // (3) for each slice, do not read in slice texture file, but directly find
    //     mesh intersection on that slice and draw path
    // REVISIT - the slice on each end of the mesh may or may not have triangle on it
    int aNumSlices = aMaxSliceIndex - aMinSliceIndex + 1;
    std::vector< cv::Mat > aIntrsctColor;
    for ( size_t i = 0; i < aNumSlices; ++i ) {
        aIntrsctColor.push_back( cv::Mat( aImgVol[ 0 ].cols, aImgVol[ 0 ].rows, CV_8UC3 ) );
    }
    std::vector< std::vector< pt > > aIntrsctPos( aNumSlices );

    // iterate through all the edges
    std::cout << "Calculating intersections..." << std::endl;
    std::set< cv::Vec2i, ChaoVis::EdgeCompareLess >::iterator aIter;
    for ( aIter = aMesh.fEdges.begin(); aIter != aMesh.fEdges.end(); ++aIter ) {

        pcl::PointXYZRGBNormal aV1 = aMesh.fPoints[ ( *aIter )[ 0 ] ];
        pcl::PointXYZRGBNormal aV2 = aMesh.fPoints[ ( *aIter )[ 1 ] ];

        int aStartIndx = ( int )ceil( aV1.z );
        int aEndIndx = ( int )floor( aV2.z );

        // safe net
        if ( aStartIndx < aMinSliceIndex || aEndIndx > aMaxSliceIndex - 1 ) {
            continue;
        }

        // interpolate all the intersection points
        for ( int i = aStartIndx; i <= aEndIndx; ++i ) {
    
            cv::Vec3b aPixel;
            int aRow, aCol;
            if ( fabs( aV2.z - aV1.z ) < 1e-6 ) {
                if ( fabs( aV2.z - i ) < 1e-6 ) {
                    // point 1
                    aRow = round( aV2.x );
                    aCol = round( aV2.y );

                    aPixel[ 0 ] = ( unsigned char )aV2.r;
                    aPixel[ 1 ] = ( unsigned char )aV2.g;
                    aPixel[ 2 ] = ( unsigned char )aV2.b;

                    pt aPt;
                    aPt.loc = cv::Vec2i( aRow, aCol );
                    aPt.color = ( unsigned char )aV2.b;
                    aIntrsctPos[ i - aMinSliceIndex ].push_back( aPt );
        
                    aIntrsctColor[ i - aMinSliceIndex ].at< cv::Vec3b >( aRow, aCol ) = aPixel;

                    // point 2
                    aRow = round( aV1.x );
                    aCol = round( aV1.y );

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
            double d = ( aV2.z - i ) / ( aV2.z - aV1.z );
    
            aRow = round( d * aV1.x + ( 1.0 - d ) * aV2.x );
            aCol = round( d * aV1.y + ( 1.0 - d ) * aV2.y );

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
    std::cout << std::endl;

    // (4) output all the path in each slices
    std::vector< cv::Mat >::iterator aStackIter;
    int aCnt = 0;
    char aOutputPath[128];
    char aOutputImgFileName[128];
    strcpy( aOutputPath, argv[ 3 ] );
    strcat( aOutputPath, "/proj%04d.png" );

    //PNG Compression params
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    // REVISIT - aIntrsctColor recorded the appearance of the intersections of the mesh with each slice
    //           however we don't save it out, instead we overlay the paths on each slice
    for ( aStackIter = aIntrsctColor.begin(); aStackIter != aIntrsctColor.end(); ++aStackIter, ++aCnt ) {

        // overlay the path to the original image
        std::cout << "\rOverlaying image " << aCnt << "..." << std::flush;

        cv::Mat aOverlayImg;
        aImgVol[ aCnt + aMinSliceIndex ].copyTo( aOverlayImg );
        cv::cvtColor( aOverlayImg, aOverlayImg, CV_GRAY2BGR );

        for ( std::vector< /*cv::Vec2i*/pt >::iterator aPathPtIter = aIntrsctPos[ aCnt ].begin(); 
                aPathPtIter != aIntrsctPos[ aCnt ].end(); 
                ++aPathPtIter ) {
            if ( ( *aPathPtIter ).loc[ 0 ] >= 0 && ( *aPathPtIter ).loc[ 0 ] < aOverlayImg.cols &&
                 ( *aPathPtIter ).loc[ 1 ] >= 0 && ( *aPathPtIter ).loc[ 1 ] < aOverlayImg.rows ) {
                aOverlayImg.at< cv::Vec3b >( ( *aPathPtIter ).loc[ 1 ], ( *aPathPtIter ).loc[ 0 ] ) = cv::Vec3b( 0, 0, 255 );
            }

        }

        sprintf( aOutputImgFileName, aOutputPath, aCnt + aMinSliceIndex );
        cv::imwrite( aOutputImgFileName, aOverlayImg, compression_params);
    }
    std::cout << std::endl;

    // create mp4 movie
    // note: with -pix_fmt option the generated file can have the largest compatibility, however,
    //       we have to make the dimensions to be even numbers
    // for compatible output
    // char aOutputVideoName[ 128 ];
    // strcpy( aOutputVideoName, argv[ 3 ] );
    // strcat( aOutputVideoName, "/compositeVideo.mp4" );
    // char aCmdStr[ 512 ];
    // sprintf( aCmdStr, "ffmpeg -framerate 30000/1001 -start_number %d -i %s -c:v libx264 -pix_fmt yuv420p -vf \"scale=iw*sar:ih, scale=\'w=if(lt(dar, 16/9), trunc(oh*a/2)*2, min(1920,ceil(iw/2)*2)):h=if(gte(dar, 16/9), trunc(ow/a/2)*2, min(1080,ceil(ih/2)*2))\', setsar=1\" -movflags faststart %s", aMinSliceIndex, aOutputPath, aOutputVideoName );
    // system( aCmdStr );

    return 0;
}
