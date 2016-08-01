#pragma once

#ifndef _VOLCART_SEGMENTATION_SPLINE_H_
#define _VOLCART_SEGMENTATION_SPLINE_H_

#include <unsupported/Eigen/Splines>
#include <vector>
#include <cassert>
#include "segmentation/lrps/common.h"

namespace volcart
{
namespace segmentation
{
template <typename Scalar = double, uint32_t Degree = 3>
class Spline
{
public:
    using ScalarVector = std::vector<Scalar>;
    using SplineType = Eigen::Spline<Scalar, 2>;

    Spline() = default;

    Spline(const ScalarVector& xs, const ScalarVector& ys)
    {
        assert(xs.size() == ys.size() && "xs and ys must be same length");
        npoints_ = xs.size();
        auto points = makeWideMatrix(xs, ys);
        spline_ = Eigen::SplineFitting<SplineType>::Interpolate(points, Degree);
    }

    // Plain evaluation at t-space value t \in [0, 1]
    Pixel operator()(Scalar t) const
    {
        assert(t >= 0 && t <= 1 && "t must be in range [0, 1]");
        Eigen::Vector2d val = spline_(t);
        return {val(0), val(1)};
    }

private:
    size_t npoints_;
    SplineType spline_;

    Eigen::MatrixXd makeWideMatrix(const ScalarVector& xs,
                                   const ScalarVector& ys)
    {
        Eigen::MatrixXd mat{2, xs.size()};
        mat.row(0) = Eigen::VectorXd::Map(xs.data(), xs.size());
        mat.row(1) = Eigen::VectorXd::Map(ys.data(), ys.size());
        return mat;
    }
};

template <typename Scalar>
using CubicSpline = Spline<Scalar, 3>;
}
}
#endif
