#pragma once

#ifndef _VOLCART_SEGMENTATION_FITTED_CURVE_H_
#define _VOLCART_SEGMENTATION_FITTED_CURVE_H_

#include <cassert>
#include "spline.h"
#include "common.h"

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
    vec<double> resampledPoints_;

public:
    FittedCurve() = default;

    FittedCurve(const vec<Voxel>& vs, const int32_t zIndex,
                const double resamplePerc = 1.0);

    int32_t size() const { return npoints_; }
    const decltype(spline_)& spline() const { return spline_; }
    vec<Voxel> resampledPoints() const;

    Pixel eval(double t) const { return spline_.eval(t); }
    void resample(const double resamplePerc);

    Voxel operator()(const int32_t index) const;

    Voxel derivAt(const int32_t index, const int32_t hstep = 1) const;

    vec<double> deriv(const int32_t hstep = 1) const;

    Voxel derivCentralDifference(const int32_t index,
                                 const int32_t hstep = 1) const;

    Voxel derivBackwardDifference(const int32_t index,
                                  const int32_t hstep = 1) const;

    Voxel derivForwardDifference(const int32_t index,
                                 const int32_t hstep = 1) const;

    Voxel derivFivePointStencil(const int32_t index,
                                const int32_t hstep = 1) const;
};
}
}

#endif
