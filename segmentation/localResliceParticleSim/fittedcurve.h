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

    FittedCurve(const std::vector<Voxel>& vs, int32_t zIndex);

    int32_t size(void) const { return npoints_; }

    const std::vector<Pixel>& points() const { return points_; }

    std::vector<double> xs() const { return xs_; }

    std::vector<double> ys() const { return ys_; }

    const decltype(spline_)& spline() const { return spline_; }

    std::vector<Voxel> seedPoints() const { return seedPoints_; }

    Pixel eval(double t) const { return spline_.eval(t); }

    std::vector<Voxel> resample(double resamplePerc = 1.0);

    Voxel operator()(int32_t index) const;

    std::vector<double> curvature(int32_t hstep = 1,
                                  double scaleFactor = 1) const;
};
}
}

#endif
