// texturingUtils.h
// Chao Du 2015 Apr
#ifndef _TEXTURINGUTILS_H_
#define _TEXTURINGUTILS_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "QuickSort.h"

//#define _DEBUG

// REVISIT - NOTE - All the filtering functions return double, whose value should be within the range of 0~65535,
//           because our volume data is from 16-bit (unsigned short) grayscale images. Returning negative value means error.


enum EFilterOption {
    FilterOptionIntersection = 0,
    FilterOptionNonMaximumSuppression,
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


inline bool IsLess( const double &nV1,
                    const double &nV2 )
{
    return( nV1 < nV2 );
}

// estimate intensity of volume at particle
inline double interpolate_intensity( const cv::Vec3f                            &point,
                                                                     const std::vector< cv::Mat >       &nImgVol )
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
        nImgVol[ x_min ].at< unsigned short >( y_min, z_min ) * (1 - dx) * (1 - dy) * (1 - dz) +
        nImgVol[ x_max ].at< unsigned short >( y_min, z_min ) * dx       * (1 - dy) * (1 - dz) +
        nImgVol[ x_min ].at< unsigned short >( y_max, z_min ) * (1 - dx) * dy       * (1 - dz) +
        nImgVol[ x_min ].at< unsigned short >( y_min, z_max ) * (1 - dx) * (1 - dy) * dz +
        nImgVol[ x_max ].at< unsigned short >( y_min, z_max ) * dx       * (1 - dy) * dz +
        nImgVol[ x_min ].at< unsigned short >( y_max, z_max ) * (1 - dx) * dy       * dz +
        nImgVol[ x_max ].at< unsigned short >( y_max, z_min ) * dx       * dy       * (1 - dz) +
        nImgVol[ x_max ].at< unsigned short >( y_max, z_max ) * dx       * dy       * dz;
    
    return result;
}

// Check whether the point is local maximum
inline bool IsLocalMaximum( const cv::Vec3f              &nPoint,
                            const std::vector< cv::Mat > &nImgVol,
                            double                       nSampleInterval = 0.2 )
{
    // uniformly sample around the point in 3D and check the intensity value
    bool aIsLocalMax = true;
    cv::Vec3f aVecs[ 6 ] = { cv::Vec3f( nSampleInterval, 0, 0 ), cv::Vec3f( -nSampleInterval, 0, 0 ),
                             cv::Vec3f( 0, nSampleInterval, 0 ), cv::Vec3f( 0, -nSampleInterval, 0 ),
                             cv::Vec3f( 0, 0, nSampleInterval ), cv::Vec3f( 0, 0, -nSampleInterval ) };
    double aCurValue = interpolate_intensity( nPoint, nImgVol );

    for ( int i = 0; i < 6; ++i ) {
        if ( aCurValue < interpolate_intensity( nPoint + aVecs[ i ], nImgVol )) {
            aIsLocalMax = false;
            break;
        }
    }
    return aIsLocalMax;
}

// Sample the volume data withing ellipse region
inline void Sectioning(            double                       nSections,      // number of sections
                                   double                       range,          // range (thickness) of material
                                   const cv::Vec3f              &nCenter,       // given point
                                   const cv::Vec3f              &nMajorAxisDir, // normal
                                   const std::vector< cv::Mat > &nImgVol,       // volume data
                                   double                       nA,             // neighborhood's radius (Axis)
                                   int                          nSamplingDir,   // sampling direction
                                   double                       *nData )        // [out] samples
{
    if (nMajorAxisDir == cv::Vec3f(0,0,0)) {
      printf( "ERROR: zero normal vector.\n" );
      return;
    }

    // Normalize normal to be the length of half of the material's thickness (pixels)
    cv::Vec3f aNormalVec( nMajorAxisDir[ 0 ], nMajorAxisDir[ 1 ], nMajorAxisDir[ 2 ] );
    cv::normalize( aNormalVec, aNormalVec, 0, range / 2, cv::NORM_MINMAX, CV_32F);

    double nSampleInterval = range / nSections;
    nSampleInterval = nSampleInterval / 2; // Assume given point is in the middle of the material

    int aDataCnt = 0;
    cv::Vec3f aPos;
    cv::Vec3f aDir;

    // Loop over the number of samples required on the normal vector
    for ( int i = 0; aDataCnt < nSections; ++i ) {
        aDir = i * nSampleInterval * aNormalVec;

        // Both directions
        if ( nSamplingDir == 0 ) {
          // Calculate point as scaled and sampled normal vector + center
          aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
          aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
          aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];

          // Get interpolated intensity at point
          double tmp = interpolate_intensity( aPos, nImgVol );

          // Store point in return array
          nData[ aDataCnt ] = tmp; 
          aDataCnt++;

          // Eliminates duplicate image at starting index
          if ( i > 0 && aDataCnt < nSections ) {
            aPos[ 0 ] = nCenter[ 0 ] - aDir[ 0 ];
            aPos[ 1 ] = nCenter[ 1 ] - aDir[ 1 ];
            aPos[ 2 ] = nCenter[ 2 ] - aDir[ 2 ];

            // Get interpolated intensity at point
            double tmp = interpolate_intensity( aPos, nImgVol );

            // Store point in return array
            nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
            aDataCnt++;
          }
        }
        // Positive direction only
        else if ( nSamplingDir == 1 ) {
          // Calculate point as scaled and sampled normal vector + center
          aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
          aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
          aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];

          // Get interpolated intensity at point
          double tmp = interpolate_intensity( aPos, nImgVol );

          // Store point in return array
          nData[ aDataCnt ] = tmp;
          aDataCnt++;
        }
        // Negative direction only
        else if ( nSamplingDir == 2 ) {
          aPos[ 0 ] = nCenter[ 0 ] - aDir[ 0 ];
          aPos[ 1 ] = nCenter[ 1 ] - aDir[ 1 ];
          aPos[ 2 ] = nCenter[ 2 ] - aDir[ 2 ];

          // Get interpolated intensity at point
          double tmp = interpolate_intensity( aPos, nImgVol );

          // Store point in return array
          nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
          aDataCnt++;
        }

    } // for i
}

// Sample the volume data along the normal
inline void SamplingAlongNormal(   double                       nA,             // normal length
                                   double                       nSampleInterval,// sample interval
                                   const cv::Vec3f              &nCenter,       // center
                                   const cv::Vec3f              &nMajorAxisDir, // normal
                                   const std::vector< cv::Mat > &nImgVol,       // volume data
                                   int                          nSamplingDir,   // sampling direction
                                   double                       *nData,         // [out] samples
                                   int                          *nSize,         // [out] number of samples
                                   bool                         nWithNonMaxSuppression = false ) // ONLY FOR NON-MAXIMUM SUPPRESSION
{
    // uniformly sample along the normal
    int aSizeMajor = nA / nSampleInterval;

    if (nMajorAxisDir == cv::Vec3f(0,0,0)) {
      printf( "ERROR: zero normal vector.\n" );
      *nSize = 0;
      return;
    }

    cv::Vec3f aNormalVec( nMajorAxisDir[ 0 ], nMajorAxisDir[ 1 ], nMajorAxisDir[ 2 ] );
    cv::normalize( aNormalVec, aNormalVec );

    int aDataCnt = 0;
    cv::Vec3f aPos;
    cv::Vec3f aDir;

    // Loop over the number of samples required on the normal vector
    for ( int i = 0; i < aSizeMajor; ++i ) {
        aDir = i * nSampleInterval * aNormalVec;

        if ((nSamplingDir == 0) || (nSamplingDir == 1)) {
          // Calculate point as scaled and sampled normal vector + center
          aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
          aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
          aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];

          // Get interpolated intensity at point
          double tmp = interpolate_intensity( aPos, nImgVol );

          // Store point in return array
          nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
          aDataCnt++;
        }

        if ((nSamplingDir == 0) || (nSamplingDir == 2)) {

          aPos[ 0 ] = nCenter[ 0 ] - aDir[ 0 ];
          aPos[ 1 ] = nCenter[ 1 ] - aDir[ 1 ];
          aPos[ 2 ] = nCenter[ 2 ] - aDir[ 2 ];

          // Get interpolated intensity at point
          double tmp = interpolate_intensity( aPos, nImgVol );
  
          // Store point in return array
          nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
          aDataCnt++;
        }

    } // for i

    *nSize = aDataCnt;
}

inline void SamplingWithinEllipse( double                       nA,             // normal length
                                   double                       nB,             // normal length
                                   double                       nSampleInterval,// sample interval
                                   const cv::Vec3f              &nCenter,       // center
                                   const cv::Vec3f              &nMajorAxisDir, // normal
                                   const std::vector< cv::Mat > &nImgVol,       // volume data
                                   int                          nSamplingDir,   // sampling direction
                                   double                       *nData,         // [out] samples
                                   int                          *nSize,         // [out] number of samples
                                   bool                         nWithNonMaxSuppression = false ) // ONLY FOR NON-MAXIMUM SUPPRESSION
{

    // uniformly sample within the ellipse
    // uniformly sample along the major axis
    int aSizeMajor = nA / nSampleInterval;
    int aSizeMinor = nB / nSampleInterval;

    /* vector used to parameterize surface defined by nMajorAxisDir */
    /* need to create two vectors that are orthogonal to one another */
    /* and orthogonal to nMajorAxisDir */
    cv::Vec3f G(1,0,0);

    /* project G onto plane defined by nMajorAxisDir */
    /*                                   */
    /*                       n * G       */
    /* aMinorAxisDir1 = G - ------- * n  */
    /*                       n * n       */
    /*                                   */
    cv::Vec3f aMinorAxisDir1 = G - (nMajorAxisDir.dot(G)/nMajorAxisDir.dot(nMajorAxisDir)) * nMajorAxisDir;

    /* cross product of G and nMajorAxisDir, simplifies nicely to the following */
    cv::Vec3f aMinorAxisDir2 = cv::Vec3f(0, -nMajorAxisDir(2), nMajorAxisDir(1));

    cv::normalize(aMinorAxisDir1);
    cv::normalize(aMinorAxisDir2);
    /* cv::Vec3f aMinorAxisDir2 = aMinorAxisDir1.cross( nMajorAxisDir ); */

    int aDataCnt = 0;
    cv::Vec3f aPos;
    cv::Vec3f aDir;

    int aSign[ 8 ][ 3 ] = { {  1,  1,  1 }, {  1,  1, -1 }, {  1, -1,  1 }, { -1,  1,  1 },
                            {  1, -1, -1 }, { -1,  1, -1 }, { -1, -1,  1 }, { -1, -1, -1 } };
    int aSamplingPositive[ 4 ] = { 0, 1, 2, 4 };
    int aSamplingNegative[ 4 ] = { 3, 5, 6, 7 };

    // add the points on axis first // REVISIT - not fixed yet!!!

    for ( int i = 1; i < aSizeMajor; ++i ) {
        // uniformly sample the circle slice
        for ( int j = 1; j < aSizeMinor; ++j ) {
            for ( int k = 1; k < aSizeMinor; ++k ) {
                double aEllipseDist = nB * sqrt( 1. - ( i * nSampleInterval - nA ) * ( i * nSampleInterval - nA ) );
                if ( j * nSampleInterval < aEllipseDist ) { // must satisfy ellipse constraint, (x/a)^2+(y/b)^2)=1
                    if ( nSamplingDir == 0 ) { // sample along both positive and negative directions
                        for ( int t = 0; t < 8; ++t ) {
                            aDir = i * nSampleInterval * aSign[ t ][ 0 ] * nMajorAxisDir +
                                   j * nSampleInterval * aSign[ t ][ 1 ] * aMinorAxisDir1 +
                                   k * nSampleInterval * aSign[ t ][ 2 ] * aMinorAxisDir2;
                            // REVISIT - note that the points along the axis are counted multiple times
                            //           fixed this by starting from 1 instead of 0, and add the points on axis first
                            aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
                            aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
                            aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];

                            double tmp = interpolate_intensity( aPos, nImgVol );
                            // suppress the data if it's not local maximum
                            if ( nWithNonMaxSuppression && !IsLocalMaximum( aPos, nImgVol) ) {
                                tmp = 0.0;
                            }
                            nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
                            aDataCnt++;
                        } // for t
                    } else if ( nSamplingDir == 1 ) { // sample along positive direction
                        for ( int t = 0; t < 4; ++t ) {
                            aDir = i * nSampleInterval * aSign[ aSamplingPositive[ t ] ][ 0 ] * nMajorAxisDir +
                                   j * nSampleInterval * aSign[ aSamplingPositive[ t ] ][ 1 ] * aMinorAxisDir1 +
                                   k * nSampleInterval * aSign[ aSamplingPositive[ t ] ][ 2 ] * aMinorAxisDir2;
                            aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
                            aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
                            aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];

                            double tmp = interpolate_intensity( aPos, nImgVol );
                            // suppress the data if it's not local maximum
                            if ( nWithNonMaxSuppression && !IsLocalMaximum( aPos, nImgVol) ) {
                                tmp = 0.0;
                            }
                            nData[ aDataCnt ] = tmp; // REVISIT - we assume we have enough space
                            aDataCnt++;
                        } // for t
                    } else if ( nSamplingDir == 2 ) { // sample along negative direction
                        for ( int t = 0; t < 4; ++t ) {
                            aDir = i * nSampleInterval * aSign[ aSamplingNegative[ t ] ][ 0 ] * nMajorAxisDir +
                                   j * nSampleInterval * aSign[ aSamplingNegative[ t ] ][ 1 ] * aMinorAxisDir1 +
                                   k * nSampleInterval * aSign[ aSamplingNegative[ t ] ][ 2 ] * aMinorAxisDir2;
                            aPos[ 0 ] = nCenter[ 0 ] + aDir[ 0 ];
                            aPos[ 1 ] = nCenter[ 1 ] + aDir[ 1 ];
                            aPos[ 2 ] = nCenter[ 2 ] + aDir[ 2 ];

                            double tmp = interpolate_intensity( aPos, nImgVol );
                            // suppress the data if it's not local maximum
                            if ( nWithNonMaxSuppression && !IsLocalMaximum( aPos, nImgVol) ) {
                                tmp = 0.0;
                            }
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
}

// Filter by returning the color at the point location
inline double FilterIntersection( const cv::Vec3f              &nPoint,
                                  const std::vector< cv::Mat > &nImgVol )
{
/* interpolate_intensity already does a better check for whether the point exists within the volume
 * Commenting this out in lieu of deletion. - SP
    if ( nPoint[ 0 ] < 0 || nPoint[ 0 ] > nImgVol.size() - 1 ||
         nPoint[ 1 ] < 0 || nPoint[ 1 ] > nImgVol[ 0 ].cols - 1 ||
         nPoint[ 2 ] < 0 || nPoint[ 2 ] > nImgVol[ 0 ].rows - 1 ) {

        return -1.0;

    }
*/

    return interpolate_intensity( nPoint, nImgVol );
}

// Filter by non maximum suppression
inline double FilterNonMaximumSuppression( const cv::Vec3f              &nPoint,    // point location
                                           const cv::Vec3f              &nNormal,   // point normal direction
                                           const std::vector< cv::Mat > &nImgVol,   // data volume
                                           double                       nR1 = 3.0,  // sample region radius 1, major axis
                                           double                       nR2 = 1.0,  // sample region radius 2, minor axis
                                           double                       nSampleDist = 0.2, // interval between samples
                                           EDirectionOption             nSamplingDir = DirectionOptionBoth ) // sample direction
{
    const int  MAX_ARRAY_CAPACITY = 50000;
    double     *aSamples = new double[ MAX_ARRAY_CAPACITY ];
    int        aNumSamples = -1;
    double     aResult = -1.0;

    SamplingAlongNormal( nR1,
                         nSampleDist,
                         nPoint,
                         nNormal,
                         nImgVol,
                         nSamplingDir,
                         aSamples,
                         &aNumSamples,
                         true );

    // find maximum
    if ( aNumSamples > 0 ) {
        aResult = aSamples[ 0 ];
        for ( int i = 1; i < aNumSamples; ++i ) {
            if ( aSamples[ i ] > aResult ) {
                aResult = aSamples[ i ];
            }
        }
    }

    // clean up
    if ( aSamples != NULL ) {
        delete []aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the maximum
inline double FilterMax( const cv::Vec3f              &nPoint,    // point location
                         const cv::Vec3f              &nNormal,   // point normal direction
                         const std::vector< cv::Mat > &nImgVol,   // data volume
                         double                       nR1 = 3.0,  // sample region radius 1, major axis
                         double                       nR2 = 1.0,  // sample region radius 2, minor axis
                         double                       nSampleDist = 0.2, // interval between samples
                         EDirectionOption             nSamplingDir = DirectionOptionBoth ) // sample direction
{
    const int  MAX_ARRAY_CAPACITY = 50000;
    double     *aSamples = new double[ MAX_ARRAY_CAPACITY ];
    int        aNumSamples = -1;
    double     aResult = -1.0;

    // sampling
    SamplingAlongNormal( nR1,
                         nSampleDist,
                         nPoint,
                         nNormal,
                         nImgVol,
                         nSamplingDir,
                         aSamples,
                         &aNumSamples,
                         false );

    // find maximum
    if ( aNumSamples > 0 ) {
        aResult = aSamples[ 0 ];
        for ( int i = 1; i < aNumSamples; ++i ) {
            if ( aSamples[ i ] > aResult ) {
                aResult = aSamples[ i ];
            }
        }
    }

    // clean up
    if ( aSamples != NULL ) {
        delete []aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the minimum
inline double FilterMin( const cv::Vec3f              &nPoint,    // point location
                         const cv::Vec3f              &nNormal,   // point normal direction
                         const std::vector< cv::Mat > &nImgVol,   // data volume
                         double                       nR1 = 3.0,  // sample region radius 1, major axis
                         double                       nR2 = 1.0,  // sample region radius 2, minor axis
                         double                       nSampleDist = 0.2, // interval between samples
                         EDirectionOption             nSamplingDir = DirectionOptionBoth ) // sample direction

{
    const int  MAX_ARRAY_CAPACITY = 50000;
    double     *aSamples = new double[ MAX_ARRAY_CAPACITY ];
    int        aNumSamples = -1;
    double     aResult = -1.0;

    // sampling
    SamplingAlongNormal( nR1,
                         nSampleDist,
                         nPoint,
                         nNormal,
                         nImgVol,
                         nSamplingDir,
                         aSamples,
                         &aNumSamples,
                         false );

    // find minimum
    if ( aNumSamples > 0 ) {
        aResult = aSamples[ 0 ];
        for ( int i = 1; i < aNumSamples; ++i ) {
            if ( aSamples[ i ] < aResult ) {
                aResult = aSamples[ i ];
            }
        }
    }

    // clean up
    if ( aSamples != NULL ) {
        delete []aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the median, then do the averaging
inline double FilterMedianAverage( const cv::Vec3f              &nPoint,    // point location
                                   const cv::Vec3f              &nNormal,   // point normal direction
                                   const std::vector< cv::Mat > &nImgVol,   // data volume
                                   double                       nR1 = 3.0,  // sample region radius 1, major axis
                                   double                       nR2 = 1.0,  // sample region radius 2, minor axis
                                   double                       nSampleDist = 0.2, // interval between samples
                                   EDirectionOption             nSamplingDir = DirectionOptionBoth ) // sample direction
{
    const int  MAX_ARRAY_CAPACITY = 50000;
    double     *aSamples = new double[ MAX_ARRAY_CAPACITY ];
    int        aNumSamples = -1;
    double     aResult = -1.0;

    // sampling
    SamplingAlongNormal( nR1,
                         nSampleDist,
                         nPoint,
                         nNormal,
                         nImgVol,
                         nSamplingDir,
                         aSamples,
                         &aNumSamples,
                         false );

    // find median
    if ( aNumSamples > 0 ) {
        double *aData = new double[ aNumSamples ];
        memcpy( aData, aSamples, sizeof( double ) * aNumSamples );
        // sort the data array
        ChaoVis::QuickSort< double >( aData,
                                      0,
                                      aNumSamples - 1,
                                      IsLess );

        // use the average of 10% of the data array
        int aRange = aNumSamples * 0.05;
        int aCenter = aNumSamples / 2;
        aResult = aData[ aCenter ];
        int aCnt = 1;
        for ( int i = 1; i <= aRange; ++i ) {
            aResult += aData[ aCenter + i ];
            aResult += aData[ aCenter - i ];
            aCnt += 2;
        }

        aResult /= aCnt;

        // clean up
        delete []aData;
    }


    // clean up
    if ( aSamples != NULL ) {
        delete []aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the median
inline double FilterMedian( const cv::Vec3f              &nPoint,    // point location
                            const cv::Vec3f              &nNormal,   // point normal direction
                            const std::vector< cv::Mat > &nImgVol,   // data volume
                            double                       nR1 = 3.0,  // sample region radius 1, major axis
                            double                       nR2 = 1.0,  // sample region radius 2, minor axis
                            double                       nSampleDist = 0.2, // interval between samples
                            EDirectionOption             nSamplingDir = DirectionOptionBoth ) // sample direction
{
    const int  MAX_ARRAY_CAPACITY = 50000;
    double     *aSamples = new double[ MAX_ARRAY_CAPACITY ];
    int        aNumSamples = -1;
    double     aResult = -1.0;

    // sampling
    SamplingAlongNormal( nR1,
                         nSampleDist,
                         nPoint,
                         nNormal,
                         nImgVol,
                         nSamplingDir,
                         aSamples,
                         &aNumSamples,
                         false );

    // find median
    if ( aNumSamples > 0 ) {
        double *aData = new double[ aNumSamples ];
        memcpy( aData, aSamples, sizeof( double ) * aNumSamples );
        // sort the data array
        ChaoVis::QuickSort< double >( aData,
                                      0,
                                      aNumSamples - 1,
                                      IsLess );

        aResult = aData[ aNumSamples / 2 ];

        // clean up
        delete []aData;
    }


    // clean up
    if ( aSamples != NULL ) {
        delete []aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by calculating the mean
inline double FilterMean( const cv::Vec3f              &nPoint,    // point location
                          const cv::Vec3f              &nNormal,   // point normal direction
                          const std::vector< cv::Mat > &nImgVol,   // data volume
                          double                       nR1 = 3.0,  // sample region radius 1, major axis
                          double                       nR2 = 1.0,  // sample region radius 2, minor axis
                          double                       nSampleDist = 0.2, // interval between samples
                          EDirectionOption             nSamplingDir = DirectionOptionBoth ) // sample direction
{
    const int  MAX_ARRAY_CAPACITY = 50000;
    double     *aSamples = new double[ MAX_ARRAY_CAPACITY ];
    int        aNumSamples = -1;
    double     aResult = -1.0;

    // sampling
    SamplingAlongNormal( nR1,
                         nSampleDist,
                         nPoint,
                         nNormal,
                         nImgVol,
                         nSamplingDir,
                         aSamples,
                         &aNumSamples,
                         false );

    // calculate mean
    if ( aNumSamples > 0 ) {
        aResult = 0.0;
        for ( int i = 0; i < aNumSamples; ++i ) {
            aResult += aSamples[ i ];
        }
        aResult /= aNumSamples;
    }


    // clean up
    if ( aSamples != NULL ) {
        delete []aSamples;
        aSamples = NULL;
    }

    return aResult;
}

inline double textureWithMethod(   const cv::Vec3f              &nPoint,    // point location
                                   const cv::Vec3f              &nNormal,   // point normal direction
                                   const std::vector< cv::Mat > &nImgVol,   // data volume
                                   EFilterOption                nFilter,    // filter option
                                   double                       nR1 = 3.0,  // sample region radius 1, major axis
                                   double                       nR2 = 1.0,  // sample region radius 2, minor axis
                                   double                       nSampleDist = 0.2, // interval between samples
                                   EDirectionOption             nSamplingDir = DirectionOptionBoth ) // sample direction
{
    switch ( nFilter ) {
    case EFilterOption::FilterOptionIntersection:
        return FilterIntersection( nPoint,
                                   nImgVol);

        break;
    case EFilterOption::FilterOptionMean:
        return FilterMean(  nPoint,
                            nNormal,
                            nImgVol,
                            nR1,
                            nR2,
                            nSampleDist,
                            nSamplingDir);
        break;
    case EFilterOption::FilterOptionNonMaximumSuppression:
        return FilterNonMaximumSuppression(  nPoint,
                                             nNormal,
                                             nImgVol,
                                             nR1,
                                             nR2,
                                             nSampleDist,
                                             nSamplingDir);
        break;
    case EFilterOption::FilterOptionMax:
        return FilterMax(  nPoint,
                           nNormal,
                           nImgVol,
                           nR1,
                           nR2,
                           nSampleDist,
                           nSamplingDir);
        break;
    case EFilterOption::FilterOptionMin:
        return FilterMin(  nPoint,
                           nNormal,
                           nImgVol,
                           nR1,
                           nR2,
                           nSampleDist,
                           nSamplingDir);
        break;
    case EFilterOption::FilterOptionMedian:
        return FilterMax(  nPoint,
                           nNormal,
                           nImgVol,
                           nR1,
                           nR2,
                           nSampleDist,
                           nSamplingDir);
        break;
    case EFilterOption::FilterOptionMedianAverage:
        return FilterMedianAverage( nPoint,
                                    nNormal,
                                    nImgVol,
                                    nR1,
                                    nR2,
                                    nSampleDist,
                                    nSamplingDir);
        break;
    default:
        return FilterIntersection( nPoint,
                                   nImgVol);

        break;
    } // switch nFilter
}


#endif // _TEXTURINGUTILS_H_
