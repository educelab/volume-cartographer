#pragma once

#ifndef _VOLCART_LOCAL_RESLICE_SEGMENTATION_H_
#define _VOLCART_LOCAL_RESLICE_SEGMENTATION_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "volumepkg.h"
#include "chain.h"
#include "chainmesh.h"
#include "common.h"

namespace volcart
{
namespace segmentation
{
class LocalResliceSegmentation
{
public:
    LocalResliceSegmentation(VolumePkg& pkg);

    pcl::PointCloud<pcl::PointXYZRGB> segmentPath(const VoxelVec& initPath,
                                                  const double resamplePerc,
                                                  const int32_t startIndex,
                                                  const int32_t endIndex,
                                                  const int32_t stepNumLayers);

    pcl::PointCloud<pcl::PointXYZRGB> segmentLayer(
        const bool showVisualization, const int32_t startIndex,
        const int32_t endIndex, const int32_t stepNumLayers,
        const double derivativeTolerance = kDefaultDerivativeTolerance,
        const int32_t keepNumMaxima = kDefaultKeepNumMaxima,
        const int32_t numIters = kDefaultNumIters);

private:
    VolumePkg& pkg_;

    constexpr static double kDefaultDerivativeTolerance = 1e-2;
    constexpr static int32_t kDefaultKeepNumMaxima = 5;
    constexpr static int32_t kDefaultNumIters = 10;
};
}
}

#endif
