#pragma once

#ifndef _VOLCART_SEGMENTATION_FITTED_CURVE_H_
#define _VOLCART_SEGMENTATION_FITTED_CURVE_H_

#include <vector>
#include <tuple>
#include <cinttypes>
#include <cassert>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

namespace volcart {

namespace segmentation {

// Simple curve fitting using nonlinear least squares
template <typename Scalar=double, uint32_t Degree=3>
class FittedCurve {
public:
    using ScalarVector = typename std::vector<Scalar>;
    using Point = typename std::pair<Scalar, Scalar>;
    using Spline2d = Eigen::Spline<Scalar, 2>;

    FittedCurve() = default;

    // Need to do init in constructor initializer list because Spline2d does
    // not provide operator=() DUMB
    FittedCurve(const ScalarVector& xs, const ScalarVector& ys) :
        xmin_(*std::min_element(xs.begin(), xs.end())),
        xmax_(*std::max_element(xs.begin(), xs.end())),
        spline_(doInit(xs, ys)) {}

    // Evaluate the polynomial at 'x' --> returns the point (x, y) in the curve
    std::pair<Scalar, Scalar> at(Scalar x) const
    {
        auto point = spline_(uvalue(x));
        return std::make_pair(point(0), point(1));
    }

private:
    Scalar xmin_;
    Scalar xmax_;
    Spline2d spline_;

    // Normalize x to [0, 1] (required by Eigen spline interpolation)
    Scalar uvalue(Scalar x) const
    {
        return (x - xmin_) / (xmax_ - xmin_);
    }

    Spline2d doInit(const ScalarVector& xs, const ScalarVector& ys)
    {
        auto mat = makeWideMatrix(xs, ys);
        auto xvalKnots = mat.row(0).unaryExpr([=](Scalar x) { return uvalue(x); });
        return Eigen::SplineFitting<Spline2d>::Interpolate(mat, Degree, xvalKnots);
    }

    Eigen::MatrixXd makeWideMatrix(const ScalarVector& xs, const ScalarVector& ys)
    {
        assert(xs.size() == ys.size() && "xs and ys must be the same size!");
        Eigen::MatrixXd mat{2, xs.size()};
        mat.row(0) = Eigen::VectorXd::Map(xs.data(), xs.size());
        mat.row(1) = Eigen::VectorXd::Map(ys.data(), ys.size());
        return mat;
    }
};

}

}

#endif
