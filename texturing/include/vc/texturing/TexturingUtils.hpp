// texturingUtils.h
// Chao Du 2015 Apr
#pragma once

#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"

//#define _DEBUG

// REVISIT - NOTE - All the filtering functions return double, whose value
// should be within the range of 0~65535,
//           because our volume data is from 16-bit (unsigned short) grayscale
//           images. Returning negative value means error.

// Check whether the point is local maximum
bool IsLocalMaximum(
    const cv::Vec3d& nPoint, VolumePkg& volpkg, double nSampleInterval = 0.2);

// Sample the volume data withing ellipse region
void Sectioning(
    double nSections,                       // number of sections
    double range,                           // thickness of material in voxels
    const cv::Vec3d& nCenter,               // given point
    const cv::Vec3d& nMajorAxisDir,         // normal
    VolumePkg& volpkg,                      // volume package
    double /*nA*/,                          // neighborhood's radius (Axis)
    volcart::DirectionOption nSamplingDir,  // sampling direction
    double* nData);                         // [out] samples

// Sample the volume data along the normal
void SamplingAlongNormal(
    double nA,                                 // normal length
    double nSampleInterval,                    // sample interval
    const cv::Vec3d& nCenter,                  // center
    const cv::Vec3d& nMajorAxisDir,            // normal
    VolumePkg& volpkg,                         // volume package
    volcart::DirectionOption nSamplingDir,     // sampling direction
    double* nData,                             // [out] samples
    int* nSize,                                // [out] number of samples
    bool /*nWithNonMaxSuppression = false*/);  // ONLY FOR NON-MAXIMUM
                                               // SUPPRESSION
void SamplingWithinEllipse(
    double nA,                       // normal length
    double nB,                       // normal length
    double nSampleInterval,          // sample interval
    const cv::Vec3d& nCenter,        // center
    const cv::Vec3d& nMajorAxisDir,  // normal
    VolumePkg& volpkg,               // volume package
    volcart::DirectionOption nSamplingDir,
    double* nData,                         // [out] samples
    int* nSize,                            // [out] number of samples
    bool nWithNonMaxSuppression = false);  // ONLY FOR NON-MAXIMUM SUPPRESSION

// Filter by returning the color at the point location
double FilterIntersection(const cv::Vec3d& nPoint, VolumePkg& volpkg);

// Filter by non maximum suppression
double FilterNonMaximumSuppression(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& volpkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional);  // sample direction

// Filter by finding the maximum
double FilterMax(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& volpkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional);  // sample direction

// Filter by finding the minimum
double FilterMin(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& volpkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional);  // sample direction

// Filter by finding the median, then do the averaging
double FilterMedianAverage(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& volpkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional);  // sample direction

// Filter by finding the median
double FilterMedian(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& volpkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional);  // sample direction

// Filter by calculating the mean
double FilterMean(
    const cv::Vec3d& nPoint,   // point location
    const cv::Vec3d& nNormal,  // point normal direction
    VolumePkg& volpkg,         // volume package
    double nR1 = 3.0,          // sample region radius 1, major axis
    double /*nR2*/ = 1.0,      // sample region radius 2, minor axis
    double nSampleDist = 0.2,  // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional);  // sample direction

double TextureWithMethod(
    cv::Vec3d nPoint,                  // point location
    cv::Vec3d nNormal,                 // point normal direction
    VolumePkg& volpkg,                 // volume package
    volcart::CompositeOption nFilter,  // filter option
    double nR1 = 3.0,                  // sample region radius 1, major axis
    double nR2 = 1.0,                  // sample region radius 2, minor axis
    double nSampleDist = 0.2,          // interval between samples
    volcart::DirectionOption nSamplingDir =
        volcart::DirectionOption::Bidirectional);  // sample direction
