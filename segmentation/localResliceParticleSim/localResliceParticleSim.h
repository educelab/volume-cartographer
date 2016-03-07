#pragma once

#ifndef _VOLCART_LOCAL_RESLICE_SEGMENTATION_H_
#define _VOLCART_LOCAL_RESLICE_SEGMENTATION_H_

#include <iostream>
#include <pcl/point_types.h>
#include "fittedcurve.h"
#include "volumepkg.h"
#include "common.h"

namespace volcart
{
namespace segmentation
{
class LocalResliceSegmentation
{
public:
    LocalResliceSegmentation(VolumePkg& pkg) : pkg_(pkg) {}

    pcl::PointCloud<pcl::PointXYZRGB> segmentPath(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        int32_t startIndex,
        int32_t endIndex,
        int32_t numIters,
        int32_t stepNumLayers,
        double alpha,
        double k1,
        double k2,
        double beta,
        double delta,
        int32_t peakDistanceWeight,
        bool shouldIncludeMiddle,
        bool dumpVis,
        bool visualize);

private:
    VolumePkg& pkg_;

    cv::Vec3d estimateNormalAtIndex(const FittedCurve& curve, int32_t index);

    cv::Mat drawParticlesOnSlice(const FittedCurve& curve,
                                 int32_t sliceIndex,
                                 int32_t particleIndex = -1,
                                 bool showSpline = false) const;

    constexpr static double kDefaultMinEnergyGradient = 1e-7;
};
}
}

#endif
