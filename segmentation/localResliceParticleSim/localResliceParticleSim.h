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
        const vec<Voxel>& initPath, const double resamplePerc,
        const int32_t startIndex, const int32_t endIndex,
        const int32_t numIters, const int32_t stepNumLayers, const double alpha,
        const double beta, const bool dumpVis, const bool visualize,
        const int32_t visIndex);

private:
    VolumePkg& pkg_;

    cv::Vec3d estimateNormalAtIndex(const FittedCurve& curve,
                                    const int32_t index);

    cv::Mat drawParticlesOnSlice(const vec<Voxel>& vs, const int32_t sliceIndex,
                                 const int32_t particleIndex) const;

    constexpr static double kDefaultDerivativeTolerance = 1e-2;
    constexpr static int32_t kDefaultNumIters = 10;
};
}
}

#endif
