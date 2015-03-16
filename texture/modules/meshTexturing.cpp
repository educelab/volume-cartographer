// meshTexturing.cpp
// Chao Du 2015 Mar
#include "meshTexturing.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include <string>

#include "../QuickSort.h"

#ifdef _NEED_DENOISE_
#include "denoiose_tvl1.h"
#endif // _NEED_DENOISE_

//#define gNumHistBin	65536//256 // REVISIT - gNumHistBin = 1 << 8 * sizeof( typename )
unsigned int gNumHistBin;


// estimate intensity of volume at particle
inline
double interpolate_intensity( const cv::Vec3f                    &point,
                                const std::vector< cv::Mat >    &nImgVol )
{
  double dx, dy, dz, int_part;
  // for new data
  dx = modf(point[(0)], &int_part);
  dy = modf(point[(2)], &int_part);
  dz = modf(point[(1)], &int_part);

  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(0) - 1; // REVISIT - this is because volpkg shifted the index by 1
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
    nImgVol[ x_min ].at< unsigned char >( y_min, z_min ) * (1 - dx) * (1 - dy) * (1 - dz) +
    nImgVol[ x_max ].at< unsigned char >( y_min, z_min ) * dx       * (1 - dy) * (1 - dz) +
    nImgVol[ x_min ].at< unsigned char >( y_max, z_min ) * (1 - dx) * dy       * (1 - dz) +
    nImgVol[ x_min ].at< unsigned char >( y_min, z_max ) * (1 - dx) * (1 - dy) * dz +
    nImgVol[ x_max ].at< unsigned char >( y_min, z_max ) * dx       * (1 - dy) * dz +
    nImgVol[ x_min ].at< unsigned char >( y_max, z_max ) * (1 - dx) * dy       * dz +
    nImgVol[ x_max ].at< unsigned char >( y_max, z_min ) * dx       * dy       * (1 - dz) +
    nImgVol[ x_max ].at< unsigned char >( y_max, z_max ) * dx       * dy       * dz;

  return result;
}

bool CompareXLess( const pcl::PointXYZRGBNormal &nP1,
                   const pcl::PointXYZRGBNormal &nP2 )
{
    return( nP1.x < nP2.x );
}

void FindBetterTexture( ChaoVis::CMesh &nMesh,
                        const std::vector< cv::Mat > &nImgVol,
                        float nRadius,
                        //int nStartIndex,
                        int   nSamplingDir,
                        double (*BetterTextureFunc)(double *nData, int nSize) )
{
    // find the NEAREST local maximum, with regular sampling
    // for example:
    // +--+--+--O--+--+--+
    /* /\  /\/\__  /\    / */
    /*   \/      \/  \__/  */
    // however, note that this is inaccurate, because we use trilinear interpolation for the color
    // at location not at the vertices, the intensity curve between vertices is linear
    const double SAMPLE_RATE = 0.2;
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
        cv::Vec3f aFarthest1;
        cv::Vec3f aFarthest2;

        if ( nSamplingDir == 0 ) { // sample both positive and negative direction
            aFarthest1 = aPos + nRadius * ( 1 + SAMPLE_RATE ) * aNormalVec;
            aFarthest2 = aPos - nRadius * ( 1 + SAMPLE_RATE ) * aNormalVec;
        } else if ( nSamplingDir == 1 ) { // sample along positive direction
            aFarthest1 = aPos + nRadius * ( 1 + SAMPLE_RATE ) * aNormalVec;
            aFarthest2 = aPos;
        } else if ( nSamplingDir == 2 ) { // sample along negative direction
            aFarthest1 = aPos;
            aFarthest2 = aPos - nRadius * ( 1 + SAMPLE_RATE ) * aNormalVec;
        } else {
            printf( "WARNING: invalid parameter in FindBetterTexture() nSamplingDir: %d\n", nSamplingDir );
            continue;
        }

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

void FindBetterTextureMedianFilter( ChaoVis::CMesh &nMesh,
                        const std::vector< cv::Mat > &nImgVol,
                        float nMajorAxisLen,
                        float nMinorAxisLen,
                        //int nStartIndex,
                        int   nSamplingDir,
                        double (*BetterTextureFunc)(double *nData, int nSize) )
{
    const int MAX_ARRAY_CAPACITY = 50000;
    const double SAMPLE_RATE = 0.2;
    double *aSamples = new double[ MAX_ARRAY_CAPACITY ];
    int aSize;

    std::vector< pcl::PointXYZRGBNormal >::iterator aIter;
    for ( aIter = nMesh.fPoints.begin(); aIter != nMesh.fPoints.end(); ++aIter ) {
        cv::Vec3f aNormalVec( aIter->normal[ 0 ], aIter->normal[ 1 ], aIter->normal[ 2 ] );
        cv::normalize( aNormalVec, aNormalVec );
        cv::Vec3f aPos( aIter->x, aIter->y, aIter->z );

        SamplingWithinEllipse( nMajorAxisLen,
                               nMinorAxisLen,
                               SAMPLE_RATE,
                               aPos,
                               aNormalVec,
                               nImgVol,
                               nSamplingDir,
                               aSamples,
                               &aSize );

        if ( aSize > 0 ) {
            unsigned char c = ( unsigned char )( BetterTextureFunc( aSamples, aSize ) );
            uint32_t color =
                               c |
                               c << 8 |
                           c << 16;
            aIter->rgb = *reinterpret_cast<float*>(&color);
        }
    }

    // clean up
    if ( aSamples != NULL ) {
        delete []aSamples;
        aSamples = NULL;
    }
}

// function pointer for selecting the best texture
double FilterNonMaximumSuppression( double    *nData,
                                    int        nSize )
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
//                cv::Vec3f aNewPos = aFarthest2 + index * aNormalVec * SAMPLE_RATE;
//                aIter->x = aNewPos[ 0 ];
//                aIter->y = aNewPos[ 1 ];
//                aIter->z = aNewPos[ 2 ];
                
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
/*        for ( int j = 0; j < aGlobalHist[ i ]; ++j ) {
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
        cv::Mat aOriginImg( nImgVol[ i ].rows, nImgVol[ i ].cols, CV_16UC1 );
        nImgVol[ i ].copyTo( aOriginImg );
        nImgVol[ i ].convertTo( nImgVol[ i ], CV_8U );

        for ( int row = 0; row < aOriginImg.rows; ++row ) {
            for ( int col = 0; col < aOriginImg.cols; ++col ) {
                nImgVol[ i ].at< unsigned char >( row, col ) = aOriginImg.at< unsigned short >( row, col ) / 256.0;
            }
        }
    }
}

void ProcessVolume( /*const*/ VolumePkg &nVpkg, 
                    std::vector< cv::Mat > &nImgVol,
                    bool nNeedEqualize,
                    bool nNeedNormalize )
{
    int aNumSlices = nVpkg.getNumberOfSlices();
    for ( int i = 0; i < aNumSlices; ++i ) {
        nImgVol.push_back( nVpkg.getSliceAtIndex( i ).clone() );
    }
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

void SamplingWithinEllipse( double nA, // major axis
                            double nB, // minor axis
                            double nDensity, // sample rate
                            const cv::Vec3f &nCenter, // ellipse center
                            const cv::Vec3f &nMajorAxisDir, // ellipse major axis direction
                            const std::vector< cv::Mat > &nImgVol,
                            int nSamplingDir,
                            double *nData,
                            int *nSize )
{
    // uniformly sample within the ellipse
    // uniformly sample along the major axis
    int aSizeMajor = nA / nDensity;
    int aSizeMinor = nB / nDensity;

    cv::Vec3f aMinorAxisDir1;
    if ( fabs( nMajorAxisDir[ 2 ] ) > 1e-6 ) {
        aMinorAxisDir1[ 0 ] = 0.0;
        aMinorAxisDir1[ 1 ] = 1.0;
        aMinorAxisDir1[ 2 ] = -nMajorAxisDir[ 1 ] / nMajorAxisDir[ 2 ];
    } else if ( fabs( nMajorAxisDir[ 1 ] ) > 1e-6 ) {
        aMinorAxisDir1[ 0 ] = 1.0;
        aMinorAxisDir1[ 1 ] = -nMajorAxisDir[ 0 ] / nMajorAxisDir[ 1 ];
        aMinorAxisDir1[ 2 ] = 0.0;
    } else if ( fabs( nMajorAxisDir[ 0 ] ) > 1e-6 ) {
        aMinorAxisDir1[ 0 ] = 0.0;
        aMinorAxisDir1[ 1 ] = 1.0;
        aMinorAxisDir1[ 2 ] = 0.0;
    } else {
        printf( "ERROR: zero normal vector.\n" );
        *nSize = 0;
        return;
    }
//    aMinorAxisDir1 = cv::normalize( aMinorAxisDir1 );
    cv::normalize( aMinorAxisDir1, aMinorAxisDir1 );
    cv::Vec3f aMinorAxisDir2 = aMinorAxisDir1.cross( nMajorAxisDir );

    int aDataCnt = 0;
//    std::vector< double > aDataArray;
    cv::Vec3f aPos;
    cv::Vec3f aDir;

    int aSign[ 8 ][ 3 ] = { {  1,  1,  1 }, {  1,  1, -1 }, {  1, -1,  1 }, { -1,  1,  1 },
                            {  1, -1, -1 }, { -1,  1, -1 }, { -1, -1,  1 }, { -1, -1, -1 } };
    int aSamplingPositive[ 4 ] = { 0, 1, 2, 4 };
    int aSamplingNegative[ 4 ] = { 3, 5, 6, 7 };

    // add the points on axis first
    // REVISIT - not fixed yet!!!

    for ( int i = 1; i < aSizeMajor; ++i ) {
        // uniformly sample the circle slice
        for ( int j = 1; j < aSizeMinor; ++j ) {
            for ( int k = 1; k < aSizeMinor; ++k ) {
                double aEllipseDist = nB * sqrt( 1. - ( i * nDensity - nA ) * ( i * nDensity - nA ) );
                if ( j * nDensity < aEllipseDist ) { // must satisfy ellipse constraint, (x/a)^2+(y/b)^2)=1
                    if ( nSamplingDir == 0 ) { // sample along both positive and negative directions
                        for ( int t = 0; t < 8; ++t ) {
                            aDir = i * nDensity * aSign[ t ][ 0 ] * nMajorAxisDir + 
                                   j * nDensity * aSign[ t ][ 1 ] * aMinorAxisDir1 +
                                   k * nDensity * aSign[ t ][ 2 ] * aMinorAxisDir2;
                            // REVISIT - note that the points along the axis are counted multiple times
                            //           fixed this by starting from 1 instead of 0, and add the points on axis first
                            aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
                            aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
                            aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];
                            double tmp = interpolate_intensity( aPos, nImgVol );
    //                        aDataArray.push_back( interpolate_intensity( aPos, nImgVol ) ); // REVISIT - FILL ME HERE
    //                        aDataArray.push_back( tmp ); // REVISIT - FILL ME HERE
                            // REVISIT - using vector crashes
                            nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
                            aDataCnt++;
                        } // for t
                    } else if ( nSamplingDir == 1 ) { // sample along positive direction
                        for ( int t = 0; t < 4; ++t ) {
                            aDir = i * nDensity * aSign[ aSamplingPositive[ t ] ][ 0 ] * nMajorAxisDir + 
                                   j * nDensity * aSign[ aSamplingPositive[ t ] ][ 1 ] * aMinorAxisDir1 +
                                   k * nDensity * aSign[ aSamplingPositive[ t ] ][ 2 ] * aMinorAxisDir2;
                            aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
                            aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
                            aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];
                            double tmp = interpolate_intensity( aPos, nImgVol );
                            nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
                            aDataCnt++;
                        } // for t
                    } else if ( nSamplingDir == 2 ) { // sample along negative direction
                        for ( int t = 0; t < 4; ++t ) {
                            aDir = i * nDensity * aSign[ aSamplingNegative[ t ] ][ 0 ] * nMajorAxisDir + 
                                   j * nDensity * aSign[ aSamplingNegative[ t ] ][ 1 ] * aMinorAxisDir1 +
                                   k * nDensity * aSign[ aSamplingNegative[ t ] ][ 2 ] * aMinorAxisDir2;
                            aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
                            aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
                            aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];
                            double tmp = interpolate_intensity( aPos, nImgVol );
                            nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
                            aDataCnt++;
                        } // for t
                    } else {
                        printf( "WARNING: invalid parameter in SamplingWithinEllipse: nSamplingDir %d\n", nSamplingDir );
                        return;
                    }
                } // if
            } // for k
        } // for j
    } // for i

    *nSize = aDataCnt;
/*
    for ( size_t i = 0; i < aDataArray.size(); ++i ) {
        nData[ i ] = aDataArray[ i ];
    }
*/
}

bool IsLess( const double &nV1,
             const double &nV2 )
{
    return( nV1 < nV2 );
}

double FilterMedianAverage( double *nData,
                            int    nSize )
{
    double *aData = new double[ nSize ];
    memcpy( aData, nData, sizeof( double ) * nSize );
    // sort the data array
    ChaoVis::QuickSort< double >( aData,
                                   0,
                                   nSize - 1,
                                   IsLess );

    // find the median, here we use the average of 10% of the data array
    int aRange = nSize * 0.05;
    int aCenter = nSize / 2;
    double aResult = aData[ aCenter ];
    int aCnt = 1;
    for ( int i = 1; i <= aRange; ++i ) {
        aResult += aData[ aCenter + i ]; 
        aResult += aData[ aCenter - i ]; 
        aCnt += 2;
    }

    // clean up
    delete []aData;

    return( aResult / aCnt );
}

double FilterMin( double *nData,
                  int    nSize )
{
    double aResult = nData[ 0 ];
    for ( int i = 1; i < nSize; ++i ) {
        if ( nData[ i ] < aResult ) {
            aResult = nData[ i ];
        }
    }
    return( aResult );
}

double FilterMedian( double *nData,
                     int    nSize )
{
    double *aData = new double[ nSize ];
    memcpy( aData, nData, sizeof( double ) * nSize );
    // sort the data array
    ChaoVis::QuickSort< double >( aData,
                                   0,
                                   nSize - 1,
                                   IsLess );

    double aResult = aData[ nSize / 2 ];

    // clean up
    delete []aData;

    return( aResult );
}

double FilterMean( double *nData,
                   int    nSize )
{
    double aResult = 0.0;
    for ( int i = 0; i < nSize; ++i ) {
        aResult += nData[ i ]; 
    }
    return( aResult / nSize );
}

void meshTexturing( ChaoVis::CMesh      &nMesh,     // mesh
                    /*const*/ VolumePkg     &nVpkg,     // volume package
                    double              nR1,        // radius 1
                    double              nR2,        // radius 2
                    EFilterOption       nFilter,    // filter option
                    EDirectionOption    nDir )      // direction option
{
    std::vector< cv::Mat > aImgVol;
    ProcessVolume( nVpkg, aImgVol, false, false );

    switch ( nFilter ) {
    case EFilterOption::FilterOptionIntersection:

        printf( "equalized texture\n" );
        FindBetterTexture( nMesh,
                            aImgVol,
                            nR1,//3.0,
                            nDir,
                            FilterDummy );
        break;
    case EFilterOption::FilterOptionMean:
        printf( "find better texture: mean filtering\n" );
        FindBetterTextureMedianFilter( nMesh,
                                       aImgVol,
                                       nR1,
                                       nR2,
                                       nDir,
                                       FilterMean );
        break;
    case EFilterOption::FilterOptionMax:

        printf( "find better texture: non-maximum suppression\n" );
        FindBetterTexture( nMesh,
                            aImgVol,
                            nR1,//3.0,
                            nDir,
                            FilterNonMaximumSuppression );
        break;
    case EFilterOption::FilterOptionMin:
        printf( "find better texture: minimum filtering\n" );
        FindBetterTextureMedianFilter( nMesh,
                                       aImgVol,
                                       nR1,
                                       nR2,
                                       nDir,
                                       FilterMin );
        break;
    case EFilterOption::FilterOptionMedian:
        printf( "find better texture: median filtering\n" );
        FindBetterTextureMedianFilter( nMesh,
                                       aImgVol,
                                       nR1,
                                       nR2,
                                       nDir,
                                       FilterMedian );
        break;
    case EFilterOption::FilterOptionMedianAverage:
        printf( "find better texture: median with average filtering\n" );
        FindBetterTextureMedianFilter( nMesh,
                                       aImgVol,
                                       nR1,
                                       nR2,
                                       nDir,
                                       FilterMedianAverage );
        break;
    default:
        printf( "ERROR: unknonw filter option.\n" );
        break;
    } // switch nFilter
}
