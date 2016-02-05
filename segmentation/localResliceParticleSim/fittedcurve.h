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
class FittedCurve
{
private:
    CubicSpline<double> spline_;
    int32_t npoints_;
    int32_t zIndex_;
    vec<double> resampledPoints_;

public:
    FittedCurve() = default;

    FittedCurve(const vec<Voxel>& vs, const int32_t zIndex)
        : npoints_(vs.size()), zIndex_(zIndex)
    {
        vec<double> xs, ys;
        xs.reserve(vs.size());
        ys.reserve(vs.size());
        for (const Voxel v : vs) {
            xs.push_back(double(v(0)));
            ys.push_back(double(v(1)));
        }
        spline_ = CubicSpline<double>(xs, ys);
        resample(npoints_);
    }

    int32_t size() const { return npoints_; }
    const decltype(spline_)& spline() const { return spline_; }
    vec<Voxel> resampledPoints() const
    {
        vec<Voxel> rs;
        rs.reserve(npoints_);
        std::transform(resampledPoints_.begin(), resampledPoints_.end(),
                       std::back_inserter(rs), [this](double x) -> Voxel {
                           auto p = spline_.eval(x);
                           return {p(0), p(1), double(zIndex_)};
                       });
        return rs;
    }

    Pixel eval(double t) const { return spline_.eval(t); }
    void resample(double resamplePerc)
    {
        resampledPoints_.clear();
        npoints_ = std::round(resamplePerc * npoints_);
        std::cout << "npoints: " << npoints_ << std::endl;
        resampledPoints_.resize(npoints_, 0.0);

        // Calculate new knot positions in t-space
        double sum = 0;
        for (int32_t i = 0; i < npoints_ && sum <= 1;
             ++i, sum += 1.0 / (npoints_ - 1)) {
            resampledPoints_[i] = sum;
        }
    }

    Voxel operator()(const int32_t index) const
    {
        auto t = resampledPoints_[index];
        Pixel p = spline_.eval(t);
        return {p(0), p(1), double(zIndex_)};
    }

    Voxel derivAt(const int32_t index, const int32_t hstep = 1) const
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

    vec<double> deriv(const int32_t hstep = 1) const
    {
        vec<double> ps;
        ps.reserve(npoints_);
        for (int32_t i = 0; i < npoints_; ++i) {
            ps.push_back(derivAt(i, hstep)(1));
        }
        return ps;
    }

    Voxel derivCentralDifference(int32_t index, int32_t hstep = 1) const
    {
        assert(index >= 1 && index <= int32_t(resampledPoints_.size() - 1) &&
               "index must not be an endpoint\n");
        double before = resampledPoints_[index - hstep];
        double after = resampledPoints_[index + hstep];
        auto p = spline_.eval(after) - spline_.eval(before);
        return {p(0), p(1), double(zIndex_)};
    }

    Voxel derivBackwardDifference(int32_t index, int32_t hstep = 1) const
    {
        assert(index >= 1 && "index must not be first point\n");
        double current = resampledPoints_[index];
        double before = resampledPoints_[index - hstep];
        auto p = spline_.eval(current) - spline_.eval(before);
        return {p(0), p(1), double(zIndex_)};
    }

    Voxel derivForwardDifference(int32_t index, int32_t hstep = 1) const
    {
        assert(index <= int32_t(resampledPoints_.size() - 2) &&
               "index must not be last point\n");
        double current = resampledPoints_[index];
        double after = resampledPoints_[index + hstep];
        auto p = spline_.eval(after) - spline_.eval(current);
        return {p(0), p(1), double(zIndex_)};
    }

    Voxel derivFivePointStencil(int32_t index, int32_t hstep = 1) const
    {
        assert(index >= 2 * hstep &&
               index <= int32_t(resampledPoints_.size() - 2 * hstep) &&
               "index must not be first/last two points for consideration\n");
        double before2 = resampledPoints_[index - (2 * hstep)];
        double before1 = resampledPoints_[index - hstep];
        double after1 = resampledPoints_[index + hstep];
        double after2 = resampledPoints_[index + (2 * hstep)];
        auto p = -spline_.eval(after2) + 8 * spline_.eval(after1) -
                 8 * spline_.eval(before1) + spline_.eval(before2);
        return {p(0), p(1), double(zIndex_)};
    }
};
}
}

#endif
