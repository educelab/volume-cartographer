#pragma once

#ifndef _VOLCART_SEGMENTATION_FITTED_CURVE_H_
#define _VOLCART_SEGMENTATION_FITTED_CURVE_H_

#include <vector>
#include <cassert>
#include "common.h"
#include "derivative.h"
#include "spline.h"

namespace volcart
{
namespace segmentation
{
class FittedCurve
{
private:
    CubicSpline<double> spline_;
    int32_t npoints_;
    int32_t zIndex_;
    std::vector<double> ts_;
    std::vector<Pixel> points_;
    std::vector<Voxel> seedPoints_;
    std::vector<double> xs_;
    std::vector<double> ys_;

public:
    FittedCurve() = default;

    FittedCurve(const std::vector<Voxel>& vs, const int32_t zIndex);

    int32_t size() const { return npoints_; }

    const std::vector<Pixel>& points() const { return points_; }

    std::vector<double> xs(void) const { return xs_; }

    std::vector<double> ys(void) const { return ys_; }

    const decltype(spline_)& spline() const { return spline_; }

    std::vector<Voxel> seedPoints() const { return seedPoints_; }

    Pixel eval(double t) const { return spline_.eval(t); }

    std::vector<Voxel> resample(const double resamplePerc = 1.0);

    Voxel operator()(const int32_t index) const;

    std::vector<double> curvature(const int32_t hstep = 1,
                                  const double scaleFactor = 1) const;
};
}
}

#endif
