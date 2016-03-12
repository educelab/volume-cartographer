#pragma once

#ifndef _VOLCART_SEGMENTATION_SPLINE_H_
#define _VOLCART_SEGMENTATION_SPLINE_H_

#include <unsupported/Eigen/Splines>
#include <vector>
#include <cassert>
#include "common.h"

namespace volcart
{
namespace segmentation
{

template <typename Scalar = double, uint32_t Degree = 3>
class ParametricSpline
{
public:
    using ScalarVector = std::vector<Scalar>;
    using SplineType = Eigen::Spline<Scalar, 1>;

    ParametricSpline() = default;

    ParametricSpline(const ScalarVector& xs, const ScalarVector& ys)
        : npoints_(xs.size())
        , xs_(Eigen::VectorXd::Map(xs.data(), xs.size()))
        , ys_(Eigen::VectorXd::Map(ys.data(), ys.size()))
        , ts_(makeTPoints(xs, ys))
        , xSpline_(Eigen::SplineFitting<SplineType>::Interpolate(
              xs_.transpose(), Degree, ts_))
        , ySpline_(Eigen::SplineFitting<SplineType>::Interpolate(
              ys_.transpose(), Degree, ts_))
    {
    }

    // Plain evaluation at t-space value t \in [0, 1]
    Pixel operator()(Scalar t) const
    {
        assert(t >= 0 && t <= 1 && "out of bounds");
        return {xSpline_(t)(0), ySpline_(t)(0)};
    }

private:
    size_t npoints_;
    Eigen::VectorXd xs_, ys_;
    Eigen::RowVectorXd ts_;
    SplineType xSpline_;
    SplineType ySpline_;

    Eigen::RowVectorXd makeTPoints(const ScalarVector& xs,
                                   const ScalarVector& ys)
    {
        assert(xs.size() == ys.size() && "xs and ys must be same size");
        Eigen::RowVectorXd ts(xs.size());
        double sum = 0;
        for (size_t i = 1; i < xs.size(); ++i) {
            sum += std::sqrt((xs[i] - xs[i - 1]) * (xs[i] - xs[i - 1]) +
                             (ys[i] - ys[i - 1]) * (ys[i] - ys[i - 1]));
        }
        ts(0) = 0;
        for (size_t i = 1; i < xs.size(); ++i) {
            ts(i) = ts(i - 1) +
                    std::sqrt((xs[i] - xs[i - 1]) * (xs[i] - xs[i - 1]) +
                              (ys[i] - ys[i - 1]) * (ys[i] - ys[i - 1]));
        }
        return ts / sum;
    }
};

template <typename Scalar>
using CubicParametricSpline = ParametricSpline<Scalar, 3>;
}
}
#endif
