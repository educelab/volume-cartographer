#include <array>

#include <opencv2/core.hpp>

#include "vc/texturing/TexturingUtils.hpp"

bool IsLocalMaximum(
    const cv::Vec3d& nPoint, volcart::VolumePkg& volpkg, double nSampleInterval)
{
    // uniformly sample around the point in 3D and check the intensity value
    bool aIsLocalMax = true;
    std::array<cv::Vec3d, 6> aVecs = {
        cv::Vec3d(nSampleInterval, 0, 0), cv::Vec3d(-nSampleInterval, 0, 0),
        cv::Vec3d(0, nSampleInterval, 0), cv::Vec3d(0, -nSampleInterval, 0),
        cv::Vec3d(0, 0, nSampleInterval), cv::Vec3d(0, 0, -nSampleInterval)};
    double aCurValue = volpkg.volume()->interpolatedIntensityAt(nPoint);

    for (const auto& v : aVecs) {
        if (aCurValue < volpkg.volume()->interpolatedIntensityAt(nPoint + v)) {
            aIsLocalMax = false;
            break;
        }
    }
    return aIsLocalMax;
}

void Sectioning(
    double nSections,
    double range,
    const cv::Vec3d& nCenter,
    const cv::Vec3d& nMajorAxisDir,
    volcart::VolumePkg& volpkg,
    double /*nA*/,
    volcart::DirectionOption nSamplingDir,
    double* nData)
{
    if (nMajorAxisDir == cv::Vec3d(0, 0, 0)) {
        std::cerr << "ERROR: zero normal vector" << std::endl;
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
                double tmp = volpkg.volume()->interpolatedIntensityAt(aPos);

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
                        volpkg.volume()->interpolatedIntensityAt(aPos);

                    // Store point in return array
                    nData[aDataCnt] = intensity;  // REVISIT #192
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
                double tmp = volpkg.volume()->interpolatedIntensityAt(aPos);

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
                double tmp = volpkg.volume()->interpolatedIntensityAt(aPos);

                // Store point in return array
                nData[aDataCnt] = tmp;  // REVISIT - #192
                aDataCnt++;
                break;
            }
        }
    }  // for i
}

void SamplingAlongNormal(
    double nA,                                // normal length
    double nSampleInterval,                   // sample interval
    const cv::Vec3d& nCenter,                 // center
    const cv::Vec3d& nMajorAxisDir,           // normal
    volcart::VolumePkg& volpkg,               // volume package
    volcart::DirectionOption nSamplingDir,    // sampling direction
    double* nData,                            // [out] samples
    int* nSize,                               // [out] number of samples
    bool /*nWithNonMaxSuppression = false*/)  // ONLY FOR NON-MAXIMUM
                                              // SUPPRESSION
{
    // uniformly sample along the normal
    int aSizeMajor = static_cast<int>(nA / nSampleInterval);

    if (nMajorAxisDir == cv::Vec3d(0, 0, 0)) {
        std::cerr << " ERROR: zero normal vector" << std::endl;
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
            double tmp = volpkg.volume()->interpolatedIntensityAt(aPos);

            // Store point in return array
            nData[aDataCnt] = tmp;  // REVISIT - #192
            aDataCnt++;
        }

        if (nSamplingDir == volcart::DirectionOption::Bidirectional ||
            nSamplingDir == volcart::DirectionOption::Negative) {
            aPos[0] = nCenter[0] - aDir[0];
            aPos[1] = nCenter[1] - aDir[1];
            aPos[2] = nCenter[2] - aDir[2];

            // Get interpolated intensity at point
            double tmp = volpkg.volume()->interpolatedIntensityAt(aPos);

            // Store point in return array
            nData[aDataCnt] = tmp;  // REVISIT - #192
            aDataCnt++;
        }

    }  // for i

    *nSize = aDataCnt;
}

void SamplingWithinEllipse(
    double nA,                       // normal length
    double nB,                       // normal length
    double nSampleInterval,          // sample interval
    const cv::Vec3d& nCenter,        // center
    const cv::Vec3d& nMajorAxisDir,  // normal
    volcart::VolumePkg& volpkg,      // volume package
    volcart::DirectionOption nSamplingDir,
    double* nData,                // [out] samples
    int* nSize,                   // [out] number of samples
    bool nWithNonMaxSuppression)  // ONLY FOR NON-MAXIMUM SUPPRESSION
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
        G - (nMajorAxisDir.dot(G) / nMajorAxisDir.dot(nMajorAxisDir)) *
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

    // add the points on axis first // REVISIT - #193

    for (int i = 1; i < aSizeMajor; ++i) {
        // uniformly sample the circle slice
        for (int j = 1; j < aSizeMinor; ++j) {
            for (int k = 1; k < aSizeMinor; ++k) {
                auto aEllipseDist =
                    nB * sqrt(
                             1. - (i * nSampleInterval - nA) *
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
                                // REVISIT (Issue #193) - note that the points
                                //           along the axis are counted multiple
                                //           times fixed this by starting from 1
                                //           instead of 0, and add the points on
                                //           axis first
                                aPos[0] = nCenter[0] + aDir[0];
                                aPos[1] = nCenter[1] + aDir[1];
                                aPos[2] = nCenter[2] + aDir[2];

                                double tmp =
                                    volpkg.volume()->interpolatedIntensityAt(
                                        aPos);
                                // suppress the data if it's not local maximum
                                if (nWithNonMaxSuppression &&
                                    !IsLocalMaximum(aPos, volpkg)) {
                                    tmp = 0.0;
                                }
                                nData[aDataCnt] = tmp;  // REVISIT #192
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
                                    volpkg.volume()->interpolatedIntensityAt(
                                        aPos);
                                // suppress the data if it's not local maximum
                                if (nWithNonMaxSuppression &&
                                    !IsLocalMaximum(aPos, volpkg)) {
                                    tmp = 0.0;
                                }
                                nData[aDataCnt] = tmp;  // REVISIT - #192
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
                                    volpkg.volume()->interpolatedIntensityAt(
                                        aPos);
                                // suppress the data if it's not local maximum
                                if (nWithNonMaxSuppression &&
                                    !IsLocalMaximum(aPos, volpkg)) {
                                    tmp = 0.0;
                                }
                                nData[aDataCnt] = tmp;  // REVISIT #192
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
double FilterIntersection(const cv::Vec3d& nPoint, volcart::VolumePkg& volpkg)
{
    return volpkg.volume()->interpolatedIntensityAt(nPoint);
}

// Filter by non maximum suppression
double FilterNonMaximumSuppression(
    const cv::Vec3d& nPoint,     // point location
    const cv::Vec3d& nNormal,    // point normal direction
    volcart::VolumePkg& volpkg,  // volume package
    double nR1,                  // sample region radius 1, major axis
    double /*nR2*/,              // sample region radius 2, minor axis
    double nSampleDist,          // interval between samples
    volcart::DirectionOption nSamplingDir)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, volpkg, nSamplingDir, aSamples,
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
double FilterMax(
    const cv::Vec3d& nPoint,     // point location
    const cv::Vec3d& nNormal,    // point normal direction
    volcart::VolumePkg& volpkg,  // volume package
    double nR1,                  // sample region radius 1, major axis
    double /*nR2*/,              // sample region radius 2, minor axis
    double nSampleDist,          // interval between samples
    volcart::DirectionOption nSamplingDir)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, volpkg, nSamplingDir, aSamples,
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
double FilterMin(
    const cv::Vec3d& nPoint,     // point location
    const cv::Vec3d& nNormal,    // point normal direction
    volcart::VolumePkg& volpkg,  // volume package
    double nR1,                  // sample region radius 1, major axis
    double /*nR2*/,              // sample region radius 2, minor axis
    double nSampleDist,          // interval between samples
    volcart::DirectionOption nSamplingDir)  // sample direction

{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, volpkg, nSamplingDir, aSamples,
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
double FilterMedianAverage(
    const cv::Vec3d& nPoint,     // point location
    const cv::Vec3d& nNormal,    // point normal direction
    volcart::VolumePkg& volpkg,  // volume package
    double nR1,                  // sample region radius 1, major axis
    double /*nR2*/,              // sample region radius 2, minor axis
    double nSampleDist,          // interval between samples
    volcart::DirectionOption nSamplingDir)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, volpkg, nSamplingDir, aSamples,
        &aNumSamples, false);

    // find median
    if (aNumSamples > 0) {
        double* aData = new double[aNumSamples];
        memcpy(aData, aSamples, sizeof(double) * aNumSamples);
        // sort the data array
        std::sort(aData, aData + aNumSamples);

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
double FilterMedian(
    const cv::Vec3d& nPoint,     // point location
    const cv::Vec3d& nNormal,    // point normal direction
    volcart::VolumePkg& volpkg,  // volume package
    double nR1,                  // sample region radius 1, major axis
    double /*nR2*/,              // sample region radius 2, minor axis
    double nSampleDist,          // interval between samples
    volcart::DirectionOption nSamplingDir)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, volpkg, nSamplingDir, aSamples,
        &aNumSamples, false);

    // find median
    if (aNumSamples > 0) {
        double* aData = new double[aNumSamples];
        memcpy(aData, aSamples, sizeof(double) * aNumSamples);
        // sort the data array
        std::sort(aData, aData + aNumSamples);

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
double FilterMean(
    const cv::Vec3d& nPoint,     // point location
    const cv::Vec3d& nNormal,    // point normal direction
    volcart::VolumePkg& volpkg,  // volume package
    double nR1,                  // sample region radius 1, major axis
    double /*nR2*/,              // sample region radius 2, minor axis
    double nSampleDist,          // interval between samples
    volcart::DirectionOption nSamplingDir)  // sample direction
{
    const int MAX_ARRAY_CAPACITY = 50000;
    double* aSamples = new double[MAX_ARRAY_CAPACITY];
    int aNumSamples = -1;
    double aResult = -1.0;

    // sampling
    SamplingAlongNormal(
        nR1, nSampleDist, nPoint, nNormal, volpkg, nSamplingDir, aSamples,
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

double TextureWithMethod(
    cv::Vec3d nPoint,                  // point location
    cv::Vec3d nNormal,                 // point normal direction
    volcart::VolumePkg& volpkg,        // volume package
    volcart::CompositeOption nFilter,  // filter option
    double nR1,                        // sample region radius 1, major axis
    double nR2,                        // sample region radius 2, minor axis
    double nSampleDist,                // interval between samples
    volcart::DirectionOption nSamplingDir)  // sample direction
{
    switch (nFilter) {
        case volcart::CompositeOption::Intersection:
            return FilterIntersection(nPoint, volpkg);
        case volcart::CompositeOption::Mean:
            return FilterMean(
                nPoint, nNormal, volpkg, nR1, nR2, nSampleDist, nSamplingDir);
        case volcart::CompositeOption::NonMaximumSuppression:
            return FilterNonMaximumSuppression(
                nPoint, nNormal, volpkg, nR1, nR2, nSampleDist, nSamplingDir);
        case volcart::CompositeOption::Maximum:
            return FilterMax(
                nPoint, nNormal, volpkg, nR1, nR2, nSampleDist, nSamplingDir);
        case volcart::CompositeOption::Minimum:
            return FilterMin(
                nPoint, nNormal, volpkg, nR1, nR2, nSampleDist, nSamplingDir);
        case volcart::CompositeOption::Median:
            return FilterMax(
                nPoint, nNormal, volpkg, nR1, nR2, nSampleDist, nSamplingDir);
        case volcart::CompositeOption::MedianAverage:
            return FilterMedianAverage(
                nPoint, nNormal, volpkg, nR1, nR2, nSampleDist, nSamplingDir);
    }  // switch nFilter
}
