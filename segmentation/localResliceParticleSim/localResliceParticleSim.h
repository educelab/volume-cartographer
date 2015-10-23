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

namespace volcart {

namespace segmentation {

class LocalResliceSegmentation {
public:
    LocalResliceSegmentation(VolumePkg& pkg);

    pcl::PointCloud<pcl::PointXYZRGB> segmentLayer(
            const double driftTolerance,
            const int32_t startIndex=kDefaultStartIndex,
            const int32_t endIndex=kDefaultEndIndex,
            const int32_t neighborhoodRadius=kDefaultNeighborhoodRadius,
            const int32_t stepsBeforeReslice=kDefaultStepsBeforeReslice,
            const int32_t stepNumLayers=kDefaultStepNumLayers,
            const int32_t maxIterations=kDefaultMaxIterations);

    ChainMesh mesh() const { return mesh_; }

private:
    VolumePkg& pkg_;
    int32_t startIndex_;
    int32_t endIndex_;
    ChainMesh mesh_;

    std::vector<int32_t> _getNeighborIndices(
            const Chain c, const int32_t index, const int32_t neighborhoodRadius);

    constexpr static int32_t kDefaultNeighborhoodRadius = 3;
    constexpr static int32_t kDefaultStepsBeforeReslice = 1;
    constexpr static int32_t kDefaultStartIndex         = 0;
    constexpr static int32_t kDefaultEndIndex           = -1;
    constexpr static int32_t kDefaultStepNumLayers      = 1;
    constexpr static int32_t kDefaultMaxIterations      = 100;

};

}

}

#endif
