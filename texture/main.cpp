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
        std::cout << "Usage: sliceIntersection mesh.ply radius wantBetterTexture(0=no, 1=non-maximum suppression, 2=median filter) volPkgPath" << std::endl;
        exit( -1 );
    }

    radius = atof( argv[ 2 ] );

    int aFindBetterTextureMethod = atoi( argv[ 3 ] );
    // (1) read in ply mesh file
    ChaoVis::CMesh aMesh;
    std::string aFileName( argv[ 1 ] );
    ChaoVis::CPlyHelper::ReadPlyFile( aFileName, aMesh );
    std::cout << "Mesh file loaded" << std::endl;
    // REVISIT - for debug
    aMesh.Dump();

    //set output name
    std::string outputName = aFileName.substr(aFileName.find_last_of("/\\")+1);
    outputName = outputName.substr(0,outputName.find_last_of("."));

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

    VolumePkg vpkg = VolumePkg( argv[ 4 ] );
    std::vector< cv::Mat > aImgVol;
    ProcessVolume( vpkg, aImgVol, false, false );

    if ( aFindBetterTextureMethod == 1 ) { // non-maximum suppression
        printf( "find better texture: non-maximum suppression\n" );
        FindBetterTexture( aMesh,
                            aImgVol,
                            radius,//3.0,
                            FilterNonMaximumSuppression );

        printf( "writing result\n" );
        ChaoVis::CPlyHelper::WritePlyFile( outputName + "_textured.ply", aMesh );
    } else if ( aFindBetterTextureMethod == 2 ) { // median filter
        printf( "find better texture: median filtering\n" );
        FindBetterTextureMedianFilter( aMesh,
                                       aImgVol,
                                       radius,
                                       radius / 3.0,
                                       FilterMedianAverage );

        printf( "writing result\n" );
        ChaoVis::CPlyHelper::WritePlyFile( outputName + "_textured.ply", aMesh );
    } else {
        printf( "equalized texture\n" );
        FindBetterTexture( aMesh,
                            aImgVol,
                            radius,//3.0,
                            FilterDummy );
        printf( "writing result\n" );
        ChaoVis::CPlyHelper::WritePlyFile( outputName + "_textured.ply", aMesh );
    }

    return 0;
}
