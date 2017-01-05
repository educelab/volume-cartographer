// texturingUtils.h
// Chao Du 2015 Apr
#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "QuickSort.h"
#include "core/types/VolumePkg.h"
#include "core/vc_defines.h"

//#define _DEBUG

// REVISIT - NOTE - All the filtering functions return double, whose value
// should be within the range of 0~65535,
//           because our volume data is from 16-bit (unsigned short) grayscale
//           images. Returning negative value means error.

inline bool IsLess(const double& nV1, const double& nV2) { return (nV1 < nV2); }

// Check whether the point is local maximum
inline bool IsLocalMaximum(
    const cv::Vec3d& nPoint, VolumePkg& VolPkg, double nSampleInterval = 0.2)
{
    // uniformly sample around the point in 3D and check the intensity value
    bool aIsLocalMax = true;
    cv::Vec3d aVecs[6] = {
        cv::Vec3d(nSampleInterval, 0, 0), cv::Vec3d(-nSampleInterval, 0, 0),
        cv::Vec3d(0, nSampleInterval, 0), cv::Vec3d(0, -nSampleInterval, 0),
        cv::Vec3d(0, 0, nSampleInterval), cv::Vec3d(0, 0, -nSampleInterval)};
    double aCurValue = VolPkg.volume().interpolatedIntensityAt(nPoint);

    for (int i = 0; i < 6; ++i) {
        if (aCurValue <
            VolPkg.volume().interpolatedIntensityAt(nPoint + aVecs[i])) {
            aIsLocalMax = false;
            break;
        }
    }
    return aIsLocalMax;
}

// Sample the volume data withing ellipse region
inline void Sectioning(
    double nSections,                       // number of sections
    double range,                           // thickness of material in voxels
    const cv::Vec3d& nCenter,               // given point
    const cv::Vec3d& nMajorAxisDir,         // normal
    VolumePkg& VolPkg,                      // volume package
    double /*nA*/,                          // neighborhood's radius (Axis)
    volcart::DirectionOption nSamplingDir,  // sampling direction
    double* nData)                          // [out] samples
{
    if (nMajorAxisDir == cv::Vec3d(0, 0, 0)) {
        printf("ERROR: zero normal vector.\n");
        return;
    }

    // Normalize normal to be the length of half of the material's thickness
    // (pixels)
    cv::Vec3d aNormalVec(nMajorAxisDir[0], nMajorAxisDir[1], nMajorAxisDir[2]);
    cv::normalize(
        aNormalVec, aNormalVec, 0, range / 2, cv::NORM_MINMAX, CV_32F);

    double nSampleInterval = range / nSections;
    nSampleInterval = nSampleInterval /
                      2;  // Assume given point is in the middle of the material

    int aDataCnt = 0;
    cv::Vec3d aPos;
    cv::Vec3d aDir;

    // Loop over the number of samples required on the normal vector
    for (int i = 0; aDataCnt < nSections; ++i) {
        aDir = i * nSampleInterval * aNormalVec;

        // Both directions
        switch (nSamplingDir) {
            case volcart::DirectionOption::Bidirectional: {
                // Calculate point as scaled and sampled normal vector + center
                aPos[0] = nCenter[0] + aDir[0];
                aPos[1] = nCenter[1] + aDir[1];
                aPos[2] = nCenter[2] + aDir[2];

                // Get interpolated intensity at point
                double tmp = VolPkg.volume().interpolatedIntensityAt(aPos);

                // Store point in return array
                nData[aDataCnt] = tmp;
                aDataCnt++;

                // Eliminates duplicate image at starting index
                if (i > 0 && aDataCnt < nSections) {
                    aPos[0] = nCenter[0] - aDir[0];
                    aPos[1] = nCenter[1] - aDir[1];
                    aPos[2] = nCenter[2] - aDir[2];

                    // Get interpolated intensity at point
                    double intensity =
                        VolPkg.volume().interpolatedIntensityAt(aPos);

                    // Store point in return array
                    nData[aDataCnt] =
                        intensity;  // REVISIT - we assume we have enough space
                    aDataCnt++;
                }
                break;
            }
            case volcart::DirectionOption::Positive: {
                // Calculate point as scaled and sampled normal vector + center
                aPos[0] = nCenter[0] + aDir[0];
                aPos[1] = nCenter[1] + aDir[1];
                aPos[2] = nCenter[2] + aDir[2];

                // Get interpolated intensity at point
                double tmp = VolPkg.volume().interpolatedIntensityAt(aPos);

                // Store point in return array
                nData[aDataCnt] = tmp;
                aDataCnt++;
                break;
            }
            case volcart::DirectionOption::Negative: {
                aPos[0] = nCenter[0] - aDir[0];
                aPos[1] = nCenter[1] - aDir[1];
                aPos[2] = nCenter[2] - aDir[2];

                // Get interpolated intensity at point
                double tmp = VolPkg.volume().interpolatedIntensityAt(aPos);

                // Store point in return array
                nData[aDataCnt] =
                    tmp;  // REVISIT - we assume we have enough space
                aDataCnt++;
                break;
            }
        }
    }  // for i
}

// Sample the volume data along the normal
inline void SamplingAlongNormal(
    double nA,                                // normal length
    double nSampleInterval,                   // sample interval
    const cv::Vec3d& nCenter,                 // center
    const cv::Vec3d& nMajorAxisDir,           // normal
    VolumePkg& VolPkg,                        // volume package
    volcart::DirectionOption nSamplingDir,    // sampling direction
    double* nData,                            // [out] samples
    int* nSize,                               // [out] number of samples
    bool /*nWithNonMaxSuppression = false*/)  // ONLY FOR NON-MAXIMUM
                                              // SUPPRESSION
{
    // uniformly sample along the normal
    int aSizeMajor = static_cast<int>(nA / nSampleInterval);

    if (nMajorAxisDir == cv::Vec3d(0, 0, 0)) {
        printf("ERROR: zero normal vector.\n");
        *nSize = 0;
        return;
    }

    cv::Vec3d aNormalVec(nMajorAxisDir[0], nMajorAxisDir[1], nMajorAxisDir[2]);
    cv::normalize(aNormalVec, aNormalVec);

    int aDataCnt = 0;
    cv::Vec3d aPos;
    cv::Vec3d aDir;

    // Loop over the number of samples required on the normal vector
    for (int i = 0; i < aSizeMajor; ++i) {
        aDir = i * nSampleInterval * aNormalVec;

        if (nSamplingDir == volcart::DirectionOption::Bidirectional ||
            nSamplingDir == volcart::DirectionOption::Positive) {
            // Calculate point as scaled and sampled normal vector + center
            aPos[0] = nCenter[0] + aDir[0];
            aPos[1] = nCenter[1] + aDir[1];
            aPos[2] = nCenter[2] + aDir[2];

            // Get interpolated intensity at point
            double tmp = VolPkg.volume().interpolatedIntensityAt(aPos);

            // Store point in return array
            nData[aDataCnt] = tmp;  // REVISIT - we assume we have enough space
            aDataCnt++;
        }

        if (nSamplingDir == volcart::DirectionOption::Bidirectional ||
            nSamplingDir == volcart::DirectionOption::Negative) {
            aPos[0] = nCenter[0] - aDir[0];
            aPos[1] = nCenter[1] - aDir[1];
            aPos[2] = nCenter[2] - aDir[2];

            // Get interpolated intensity at point
            double tmp = VolPkg.volume().interpolatedIntensityAt(aPos);

            // Store point in return array
            nData[aDataCnt] = tmp;  // REVISIT - we assume we have enough space
            aDataCnt++;
        }

    }  // for i

    *nSize = aDataCnt;
}

inline void SamplingWithinEllipse(
    double nA,                       // normal length
    double nB,                       // normal length
    double nSampleInterval,          // sample interval
    const cv::Vec3d& nCenter,        // center
    const cv::Vec3d& nMajorAxisDir,  // normal
    VolumePkg& VolPkg,               // volume package
    volcart::DirectionOption nSamplingDir,
    double* nData,                        // [out] samples
    int* nSize,                           // [out] number of samples
    bool nWithNonMaxSuppression = false)  // ONLY FOR NON-MAXIMUM SUPPRESSION
{

    // uniformly sample within the ellipse
    // uniformly sample along the major axis
    int aSizeMajor = static_cast<int>(nA / nSampleInterval);
    int aSizeMinor = static_cast<int>(nB / nSampleInterval);

    /* vector used to parameterize surface defined by nMajorAxisDir */
    /* need to create two vectors that are orthogonal to one another */
    /* and orthogonal to nMajorAxisDir */
    cv::Vec3d G(0, 0, 1);

    /* project G onto plane defined by nMajorAxisDir */
    /*                                   */
    /*                       n * G       */
    /* aMinorAxisDir1 = G - ------- * n  */
    /*                       n * n       */
    /*                                   */
    cv::Vec3d aMinorAxisDir1 =
        G -
        (nMajorAxisDir.dot(G) / nMajorAxisDir.dot(nMajorAxisDir)) *
            nMajorAxisDir;

    /* cross product of G and nMajorAxisDir, simplifies nicely to the following
     */
    cv::Vec3d aMinorAxisDir2 =
        cv::Vec3d(0, -nMajorAxisDir(2), nMajorAxisDir(1));

    cv::normalize(aMinorAxisDir1);
    cv::normalize(aMinorAxisDir2);
    /* cv::Vec3d aMinorAxisDir2 = aMinorAxisDir1.cross( nMajorAxisDir ); */

    int aDataCnt = 0;
    cv::Vec3d aPos;
    cv::Vec3d aDir;

    int aSign[8][3] = {{1, 1, 1},   {1, 1, -1},  {1, -1, 1},  {-1, 1, 1},
                       {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};
    int aSamplingPositive[4] = {0, 1, 2, 4};
    int aSamplingNegative[4] = {3, 5, 6, 7};

    // add the points on axis first // REVISIT - not fixed yet!!!

    for (int i = 1; i < aSizeMajor; ++i) {
        // uniformly sample the circle slice
        for (int j = 1; j < aSizeMinor; ++j) {
            for (int k = 1; k < aSizeMinor; ++k) {
                double aEllipseDist = nB * sqrt(
                                               1. -
                                               (i * nSampleInterval - nA) *
                                                   (i * nSampleInterval - nA));
                if (j * nSampleInterval <
                    aEllipseDist) {  // must satisfy ellipse constraint,
                                     // (x/a)^2+(y/b)^2)=1
                    switch (nSamplingDir) {
                        case volcart::DirectionOption::Bidirectional: {
                            for (int t = 0; t < 8; ++t) {
                                aDir = i * nSampleInterval * aSign[t][0] *
                                           nMajorAxisDir +
                                       j * nSampleInterval * aSign[t][1] *
                                           aMinorAxisDir1 +
                                       k * nSampleInterval * aSign[t][2] *
                                           aMinorAxisDir2;
                                // REVISIT - note that the points along the axis
                                // are counted multiple times
                                //           fixed this by starting from 1
                                //           instead of 0, and add the points on
                                //           axis first
                                aPos[0] = nCenter[0] + aDir[0];
                                aPos[1] = nCenter[1] + aDir[1];
                                aPos[2] = nCenter[2] + aDir[2];

                                double tmp =
                                    VolPkg.volume().interpolatedIntensityAt(
                                        aPos);
                                // suppress the data if it's not local maximum
                                if (nWithNonMaxSuppression &&
                                    !IsLocalMaximum(aPos, VolPkg)) {
                                    tmp = 0.0;
                                }
                                nData[aDataCnt] = tmp;  // REVISIT - we assume
                                                        // we have enough space
                                aDataCnt++;
                            }  // for t
                            break;
                        }
                        case volcart::DirectionOption::Positive: {
                            for (int t = 0; t < 4; ++t) {
                                aDir = i * nSampleInterval *
                                           aSign[aSamplingPositive[t]][0] *
                                           nMajorAxisDir +
                                       j * nSampleInterval *
                                           aSign[aSamplingPositive[t]][1] *
                                           aMinorAxisDir1 +
                                       k * nSampleInterval *
                                           aSign[aSamplingPositive[t]][2] *
                                           aMinorAxisDir2;
                                aPos[0] = nCenter[0] + aDir[0];
                                aPos[1] = nCenter[1] + aDir[1];
                                aPos[2] = nCenter[2] + aDir[2];

                                double tmp =
                                    VolPkg.volume().interpolatedIntensityAt(
                                        aPos);
                                // suppress the data if it's not local maximum
                                if (nWithNonMaxSuppression &&
                                    !IsLocalMaximum(aPos, VolPkg)) {
                                    tmp = 0.0;
                                }
                                nData[aDataCnt] = tmp;  // REVISIT - we assume
                                                        // we have enough space
                                aDataCnt++;
                            }  // for t
                            break;
                        }
                        case volcart::DirectionOption::Negative: {
                            for (int t = 0; t < 4; ++t) {
                                aDir = i * nSampleInterval *
                                           aSign[aSamplingNegative[t]][0] *
                                           nMajorAxisDir +
                                       j * nSampleInterval *
                                           aSign[aSamplingNegative[t]][1] *
                                           aMinorAxisDir1 +
                                       k * nSampleInterval *
                                           aSign[aSamplingNegative[t]][2] *
                                           aMinorAxisDir2;
                                aPos[0] = nCenter[0] + aDir[0];
                                aPos[1] = nCenter[1] + aDir[1];
                                aPos[2] = nCenter[2] + aDir[2];

                                double tmp =
                                    VolPkg.volume().interpolatedIntensityAt(
                                        aPos);
                                // suppress the data if it's not local maximum
                                if (nWithNonMaxSuppression &&
                                    !IsLocalMaximum(aPos, VolPkg)) {
                                    tmp = 0.0;
                                }
                                nData[aDataCnt] = tmp;  // REVISIT - we assume
                                                        // we have enough space
                                aDataCnt++;
                            }  // for t
                            break;
                        }
                    }
                }  // if
            }      // for k
        }          // for j
    }              // for i

    *nSize = aDataCnt;
}

// Filter by returning the color at the point location
inline double FilterIntersection(const cv::Vec3d& nPoint, VolumePkg& VolPkg)
{
    return VolPkg.volume().interpolatedIntensityAt(nPoint);
}

// Filter by non maximum suppression
inline double FilterNonMaximumSuppression(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& VolPkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, VolPkg, nSamplingDir, aSamples,
        &aNumSamples, true);

    // find maximum
    if (aNumSamples > 0) {
        aResult = aSamples[0];
        for (int i = 1; i < aNumSamples; ++i) {
            if (aSamples[i] > aResult) {
                aResult = aSamples[i];
            }
        }
    }

    // clean up
    if (aSamples != NULL) {
        delete[] aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the maximum
inline double FilterMax(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& VolPkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, VolPkg, nSamplingDir, aSamples,
        &aNumSamples, false);

    // find maximum
    if (aNumSamples > 0) {
        aResult = aSamples[0];
        for (int i = 1; i < aNumSamples; ++i) {
            if (aSamples[i] > aResult) {
                aResult = aSamples[i];
            }
        }
    }

    // clean up
    if (aSamples != NULL) {
        delete[] aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the minimum
inline double FilterMin(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& VolPkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional)  // sample direction

{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, VolPkg, nSamplingDir, aSamples,
        &aNumSamples, false);

    // find minimum
    if (aNumSamples > 0) {
        aResult = aSamples[0];
        for (int i = 1; i < aNumSamples; ++i) {
            if (aSamples[i] < aResult) {
                aResult = aSamples[i];
            }
        }
    }

    // clean up
    if (aSamples != NULL) {
        delete[] aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the median, then do the averaging
inline double FilterMedianAverage(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& VolPkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, VolPkg, nSamplingDir, aSamples,
        &aNumSamples, false);

    // find median
    if (aNumSamples > 0) {
        double* aData = new double[aNumSamples];
        memcpy(aData, aSamples, sizeof(double) * aNumSamples);
        // sort the data array
        ChaoVis::QuickSort<double>(aData, 0, aNumSamples - 1, IsLess);

        // use the average of 10% of the data array
        int aRange = aNumSamples / 2;
        int aCenter = aNumSamples / 2;
        aResult = aData[aCenter];
        int aCnt = 1;
        for (int i = 1; i <= aRange; ++i) {
            aResult += aData[aCenter + i];
            aResult += aData[aCenter - i];
            aCnt += 2;
        }

        aResult /= aCnt;

        // clean up
        delete[] aData;
    }

    // clean up
    if (aSamples != NULL) {
        delete[] aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by finding the median
inline double FilterMedian(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& VolPkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, VolPkg, nSamplingDir, aSamples,
        &aNumSamples, false);

    // find median
    if (aNumSamples > 0) {
        double* aData = new double[aNumSamples];
        memcpy(aData, aSamples, sizeof(double) * aNumSamples);
        // sort the data array
        ChaoVis::QuickSort<double>(aData, 0, aNumSamples - 1, IsLess);

        aResult = aData[aNumSamples / 2];

        // clean up
        delete[] aData;
    }

    // clean up
    if (aSamples != NULL) {
        delete[] aSamples;
        aSamples = NULL;
    }

    return aResult;
}

// Filter by calculating the mean
inline double FilterMean(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& VolPkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, VolPkg, nSamplingDir, aSamples,
        &aNumSamples, false);

    // calculate mean
    if (aNumSamples > 0) {
        aResult = 0.0;
        for (int i = 0; i < aNumSamples; ++i) {
            aResult += aSamples[i];
        }
        aResult /= aNumSamples;
    }

    // clean up
    if (aSamples != NULL) {
        delete[] aSamples;
        aSamples = NULL;
    }

    return aResult;
}

inline double textureWithMethod(
    const cv::Vec3d& nPoint,           // point location
    const cv::Vec3d& nNormal,          // point normal direction
    VolumePkg& VolPkg,                 // volume package
    volcart::CompositeOption nFilter,  // filter option
    double nR1 = 3.0,                  // sample region radius 1, major axis
    double nR2 = 1.0,                  // sample region radius 2, minor axis
    double nSampleDist = 0.2,          // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional)  // sample direction
{
    switch (nFilter) {
        case volcart::CompositeOption::Intersection:
            return FilterIntersection(nPoint, VolPkg);
            break;
        case volcart::CompositeOption::Mean:
            return FilterMean(
                nPoint, nNormal, VolPkg, nR1, nR2, nSampleDist, nSamplingDir);
            break;
        case volcart::CompositeOption::NonMaximumSuppression:
            return FilterNonMaximumSuppression(
                nPoint, nNormal, VolPkg, nR1, nR2, nSampleDist, nSamplingDir);
            break;
        case volcart::CompositeOption::Maximum:
            return FilterMax(
                nPoint, nNormal, VolPkg, nR1, nR2, nSampleDist, nSamplingDir);
            break;
        case volcart::CompositeOption::Minimum:
            return FilterMin(
                nPoint, nNormal, VolPkg, nR1, nR2, nSampleDist, nSamplingDir);
            break;
        case volcart::CompositeOption::Median:
            return FilterMax(
                nPoint, nNormal, VolPkg, nR1, nR2, nSampleDist, nSamplingDir);
            break;
        case volcart::CompositeOption::MedianAverage:
            return FilterMedianAverage(
                nPoint, nNormal, VolPkg, nR1, nR2, nSampleDist, nSamplingDir);
            break;
    }  // switch nFilter
}
