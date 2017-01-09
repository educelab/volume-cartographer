#pragma once

#include <iostream>

#include "core/types/OrderedPointSet.h"
#include "core/types/VolumePkg.h"
#include "segmentation/lrps/Common.h"
#include "segmentation/lrps/FittedCurve.h"

namespace volcart
{
namespace segmentation
{
class LocalResliceSegmentation
{
public:
    explicit LocalResliceSegmentation(VolumePkg& pkg) : pkg_{pkg} {}

    volcart::OrderedPointSet<cv::Vec3d> segmentPath(
        std::vector<cv::Vec3d> cloud,
        int32_t startIndex,
        int32_t endIndex,
        int32_t numIters,
        int32_t step,
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

    cv::Vec3d estimateNormalAtIndex(
        const FittedCurve& currentCurve, int32_t index);

    cv::Mat drawParticlesOnSlice(
        const FittedCurve& curve,
        int32_t sliceIndex,
        int32_t particleIndex = -1,
        bool showSpline = false) const;

    constexpr static double kDefaultMinEnergyGradient = 1e-7;
};
}
}
