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

namespace volcart {

namespace segmentation {

class LocalResliceSegmentation {
public:
    LocalResliceSegmentation(VolumePkg& pkg);

    pcl::PointCloud<pcl::PointXYZRGB> segmentLayer(
            const bool    showVisualization=kDefaultShowVisualization,
            const int32_t startIndex=kDefaultStartIndex,
            const int32_t endIndex=kDefaultEndIndex,
            const int32_t stepNumLayers=kDefaultStepNumLayers,
            const double  derivativeTolerance=kDefaultDerivativeTolerance,
            const int32_t keepNumMaxima=kDefaultKeepNumMaxima,
            const int32_t numRandomTries=kDefaultNumRandomTries);

private:
    VolumePkg& pkg_;
    int32_t startIndex_;
    int32_t endIndex_;

    double fivePointStencil(const uint32_t center, const VoxelVec& ps) const;

    constexpr static bool    kDefaultShowVisualization   = false;
    constexpr static int32_t kDefaultStartIndex          = 0;
    constexpr static int32_t kDefaultEndIndex            = -1;
    constexpr static int32_t kDefaultStepNumLayers       = 2;
    constexpr static double  kDefaultDerivativeTolerance = 1e-5;
    constexpr static int32_t kDefaultKeepNumMaxima       = 5;
    constexpr static int32_t kDefaultNumRandomTries      = 100;

};

// Some utility functions used in the algorithm that don't belong in common.h
// Returns the L2 norm of lhs - rhs
template <typename Scalar>
Scalar l2_difference_norm(const std::vector<Scalar>& lhs,
                          const std::vector<Scalar>& rhs)
{
    assert(lhs.size() == rhs.size() && "lhs and rhs must be the same size");
    Scalar out = 0;
    size_t N = lhs.size();
    for (size_t i = 0; i < N; ++i) {
        out += (lhs[i] - rhs[i]) * (lhs[i] - rhs[i]);
    }
    return std::sqrt(out);
}

}

}

#endif
