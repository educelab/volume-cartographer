// texturingTest.cpp
// Chao Du Nov 2014

#include <stdio.h>

#include "volumepkg.h"

#include "texturingUtils.h"

int main( int argc, char *argv[] )
{
    VolumePkg vpkg = VolumePkg( argv[ 1 ] );

    int aFindBetterTextureMethod = atoi( argv[ 2 ] );
    EFilterOption aFilterOption = ( EFilterOption )aFindBetterTextureMethod;

    int aSampleDir = atoi( argv[ 3 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
    EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

    std::vector< cv::Mat > aImgVol;
    int aNumSlices = vpkg.getNumberOfSlices();
    for ( int i = 0; i < aNumSlices; ++i ) {
        aImgVol.push_back( vpkg.getSliceAtIndex( i ).clone() );
    }

    double result = FilterIntersection( cv::Vec3f( 15, 29, 37 ), aImgVol );
    printf( "result is %lf\n", result );

    result = FilterMean( cv::Vec3f( 15, 29, 37 ), cv::Vec3f( 1, 0, 0 ), aImgVol );
    printf( "result is %lf\n", result );

    result = FilterMean( cv::Vec3f( 15, 29, 37 ), cv::Vec3f( 1, 0, 0 ), aImgVol );
    printf( "result is %lf\n", result );

    result = FilterMean( cv::Vec3f( 15, 29, 37 ), cv::Vec3f( 1, 0, 0 ), aImgVol );
    printf( "result is %lf\n", result );


    return 0;
}
