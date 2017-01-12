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
        int startIndex,
        int endIndex,
        int numIters,
        int step,
        double alpha,
        double k1,
        double k2,
        double beta,
        double delta,
        int peakDistanceWeight,
        bool shouldIncludeMiddle,
        bool dumpVis,
        bool visualize);

private:
    VolumePkg& pkg_;

    cv::Vec3d estimate_normal_at_index_(
        const FittedCurve& currentCurve, int index);

    cv::Mat draw_particle_on_slice_(
        const FittedCurve& curve,
        int sliceIndex,
        int particleIndex = -1,
        bool showSpline = false) const;

    constexpr static double DEFAULT_MIN_ENERGY_GRADIENT = 1e-7;
};
}
}
