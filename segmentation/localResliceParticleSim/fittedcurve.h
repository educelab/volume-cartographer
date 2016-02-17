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
    std::vector<double> tvals_;
    std::vector<Pixel> currentPoints_;
    std::vector<Voxel> seedPoints_;

public:
    FittedCurve() = default;

    FittedCurve(const std::vector<Voxel>& vs, const int32_t zIndex);

    int32_t size() const { return npoints_; }

    const std::vector<Pixel>& points() const { return currentPoints_; }

    const decltype(spline_)& spline() const { return spline_; }

    std::vector<Voxel> seedPoints() const { return seedPoints_; }

    Pixel eval(double t) const { return spline_.eval(t); }

    std::vector<Voxel> resample(const double resamplePerc = 1.0);

    Voxel operator()(const int32_t index) const;

    Pixel derivAt(const int32_t index, const int32_t hstep = 1) const;

    std::vector<Pixel> deriv(const int32_t hstep = 1) const
    {
        return d1(currentPoints_, hstep);
    }

    Pixel derivCentralDifference(const int32_t index,
                                 const int32_t hstep = 1) const
    {
        return d1Central(currentPoints_, index, hstep);
    }

    Pixel derivBackwardDifference(const int32_t index,
                                  const int32_t hstep = 1) const
    {
        return d1Backward(currentPoints_, index, hstep);
    }

    Pixel derivForwardDifference(const int32_t index,
                                 const int32_t hstep = 1) const
    {
        return d1Forward(currentPoints_, index, hstep);
    }

    Pixel derivFivePointStencil(const int32_t index,
                                const int32_t hstep = 1) const
    {
        return d1FivePointStencil(currentPoints_, index, hstep);
    }

    std::vector<double> curvature(const int32_t hstep = 1) const;
};
}
}

#endif
