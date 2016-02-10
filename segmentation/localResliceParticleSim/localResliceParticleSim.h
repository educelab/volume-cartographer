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
    pcl::PointCloud<pcl::PointXYZRGB> segmentPath(const vec<Voxel>& initPath,
                                                  const double resamplePerc,
                                                  const int32_t startIndex,
                                                  const int32_t endIndex,
                                                  const int32_t numIters,
                                                  const int32_t keepNumMaxima,
                                                  const int32_t stepNumLayers);

    /*
    pcl::PointCloud<pcl::PointXYZRGB> segmentLayer(
        const bool showVisualization, const int32_t startIndex,
        const int32_t endIndex, const int32_t stepNumLayers,
        const double derivativeTolerance = kDefaultDerivativeTolerance,
        const int32_t keepNumMaxima = kDefaultKeepNumMaxima,
        const int32_t numIters = kDefaultNumIters);
        */

private:
    VolumePkg& pkg_;

    cv::Vec3d estimateNormalAtIndex(const FittedCurve& curve,
                                    const int32_t index);

    void drawParticlesOnSlice(const vec<Voxel>& vs, const int32_t index) const;

    constexpr static double kDefaultDerivativeTolerance = 1e-2;
    constexpr static int32_t kDefaultNumIters = 10;
};
}
}

#endif
