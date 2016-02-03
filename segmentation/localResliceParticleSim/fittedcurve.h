#pragma once

#ifndef _VOLCART_SEGMENTATION_FITTED_CURVE_H_
#define _VOLCART_SEGMENTATION_FITTED_CURVE_H_

#include <vector>
#include <cassert>
#include "spline.h"
#include "common.h"

namespace volcart
{
namespace segmentation
{
template <typename Scalar>
class FittedCurve
{
private:
    CubicSpline<Scalar> spline_;
    int32_t npoints_;
    std::vector<Scalar> resampledPoints_;

public:
    using ScalarVector = std::vector<Scalar>;

    FittedCurve() = default;

    FittedCurve(const VoxelVec& vs) : npoints_(vs.size())
    {
        ScalarVector xs, ys;
        xs.reserve(vs.size());
        ys.reserve(vs.size());
        for (const Voxel v : vs) {
            xs.push_back(static_cast<Scalar>(v(0)));
            ys.push_back(static_cast<Scalar>(v(1)));
        }
        spline_ = CubicSpline<Scalar>(xs, ys);
        resample(npoints_);
    }

    const decltype(spline_)& spline() const { return spline_; }
    std::vector<Pixel> resampledPoints() const
    {
        std::vector<Pixel> rs;
        rs.reserve(npoints_);
        std::transform(resampledPoints_.begin(), resampledPoints_.end(),
                       std::back_inserter(rs),
                       [=](Scalar x) { return spline_.eval(x); });
        return rs;
    }

    Pixel eval(Scalar t) const { return spline_.eval(t); }
    void resample(size_t npoints)
    {
        resampledPoints_.clear();
        resampledPoints_.resize(npoints, 0.0);

        // Calculate new knot positions in t-space
        Scalar sum = 0;
        for (size_t i = 0; i < npoints && sum <= 1;
             ++i, sum += 1.0 / (npoints - 1)) {
            resampledPoints_[i] = sum;
        }
    }

    Pixel derivAt(const int32_t index, const int32_t hstep = 1) const
    {
        if (index - hstep <= 0) {
            return derivForwardDifference(index, hstep);
        } else if (index + hstep >= npoints_ - 1) {
            return derivBackwardDifference(index, hstep);
        } else if (index - hstep < hstep || index + hstep >= npoints_ - hstep) {
            return derivCentralDifference(index, hstep);
        } else {
            return derivFivePointStencil(index, hstep);
        }
    }

    ScalarVector deriv(const int32_t hstep = 1) const
    {
        ScalarVector ps;
        ps.reserve(npoints_);
        for (int32_t i = 0; i < npoints_; ++i) {
            ps.push_back(derivAt(i, hstep)(1));
        }
        return ps;
    }

    Pixel derivCentralDifference(int32_t index, int32_t hstep = 1) const
    {
        assert(index >= 1 &&
               index <= static_cast<int32_t>(resampledPoints_.size() - 1) &&
               "index must not be an endpoint\n");
        Scalar before = resampledPoints_[index - hstep];
        Scalar after = resampledPoints_[index + hstep];
        return spline_.eval(after) - spline_.eval(before);
    }

    Pixel derivBackwardDifference(int32_t index, int32_t hstep = 1) const
    {
        assert(index >= 1 && "index must not be first point\n");
        Scalar current = resampledPoints_[index];
        Scalar before = resampledPoints_[index - hstep];
        return spline_.eval(current) - spline_.eval(before);
    }

    Pixel derivForwardDifference(int32_t index, int32_t hstep = 1) const
    {
        assert(index <= static_cast<int32_t>(resampledPoints_.size() - 2) &&
               "index must not be last point\n");
        Scalar current = resampledPoints_[index];
        Scalar after = resampledPoints_[index + hstep];
        return spline_.eval(after) - spline_.eval(current);
    }

    Pixel derivFivePointStencil(int32_t index, int32_t hstep = 1) const
    {
        assert(index >= 2 * hstep &&
               index <=
                   static_cast<int32_t>(resampledPoints_.size() - 2 * hstep) &&
               "index must not be first/last two points for consideration\n");
        Scalar before2 = resampledPoints_[index - (2 * hstep)];
        Scalar before1 = resampledPoints_[index - hstep];
        Scalar after1 = resampledPoints_[index + hstep];
        Scalar after2 = resampledPoints_[index + (2 * hstep)];
        return -spline_.eval(after2) + 8 * spline_.eval(after1) -
               8 * spline_.eval(before1) + spline_.eval(before2);
    }
};
}
}

#endif
