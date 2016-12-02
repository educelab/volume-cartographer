#pragma once

#include <cassert>
#include <vector>
#include "segmentation/lrps/common.h"
#include "segmentation/lrps/spline.h"

namespace volcart
{
namespace segmentation
{
class FittedCurve
{
private:
    size_t npoints_;
    int32_t zIndex_;
    std::vector<double> ts_;
    std::vector<Voxel> points_;
    CubicSpline<double> spline_;

public:
    FittedCurve() : npoints_(0), zIndex_(0), ts_(), points_(), spline_() {}

    FittedCurve(const std::vector<Voxel>& vs, int32_t zIndex);

    size_t size() const { return npoints_; }

    const std::vector<Voxel>& points() const { return points_; }

    const decltype(spline_) & spline() const { return spline_; }

    Pixel eval(double t) const { return spline_(t); }

    std::vector<Voxel> evenlySpacePoints() { return resample(1.0); }

    std::vector<Voxel> resample(double resamplePerc = 1.0);

    std::vector<Voxel> sample(size_t numPoints) const;

    Voxel operator()(int32_t index) const;

    std::vector<double> curvature(int32_t hstep = 1) const;

    double arclength() const;
};
}
}
