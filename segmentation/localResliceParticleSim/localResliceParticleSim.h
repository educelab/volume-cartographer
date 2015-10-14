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

    pcl::PointCloud<pcl::PointXYZRGB> segmentLayer(const double driftTolerance,
                                                   const int32_t neighborhoodRadius=kDefaultNeighborhoodRadius,
                                                   const int32_t stepsBeforeReslice=kDefaultStepsBeforeReslice,
                                                   const int32_t startOffset=kDefaultStartOffset,
                                                   const int32_t endOffset=kDefaultEndOffset,
                                                   const int32_t stepNumLayers=kDefaultStepNumLayers,
                                                   const int32_t maxIterations=kDefaultMaxIterations);

    Chain currentChain() const { return currentChain_; }

    ChainMesh mesh() const { return mesh_; }

private:
    std::vector<int32_t> _getNeighborIndices(const int32_t index, const int32_t neighborhoodRadius);

    Chain currentChain_;
    ChainMesh mesh_;
    VolumePkg& pkg_;
    std::vector<cv::Vec3f> normals_;
    int32_t startIndex_;
    int32_t endIndex_;

    constexpr int32_t kDefaultNeighborhoodRadius = 3;
    constexpr int32_t kDefaultStepsBeforeReslice = 1;
    constexpr int32_t kDefaultStartOffset        = 0;
    constexpr int32_t kDefaultEndOffset          = 0;
    constexpr int32_t kDefaultStepNumLayers      = 1;
    constexpr int32_t kDefaultMaxIterations      = 100;

};

}

}

#endif
