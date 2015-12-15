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

    // Provide the default constructor
    FittedCurve() = default;

    FittedCurve(const ScalarVector& xs, const ScalarVector& ys) :
        xmin_(*std::min_element(xs.begin(), xs.end())),
        xmax_(*std::max_element(xs.begin(), xs.end()))
    {
        auto mat = makeWideMatrix(xs, ys);
        auto scaledXs = normalizev(mat.row(0));
        spline_ = Eigen::SplineFitting<Spline2d>::Interpolate(mat, Degree, scaledXs);
        std::cout << "ctrls:" << std::endl << spline_.ctrls() << std::endl;
        std::cout << "knots:" << std::endl << spline_.knots() << std::endl;
        std::exit(1);
    }

    // Returns point corresponding to xval as (x, y), where x in [0, 1]
    Point atReturnPoint(Scalar x) const
    {
        auto point = spline_(normalize(x));
        auto scaledX = point(0) * (xmax_ - xmin_) + xmin_;
        return std::make_pair(scaledX, point(1));
    }

    // Only returns the yval corresponding to the given xval
    Scalar at(Scalar x) const
    {
        return spline_(normalize(x))(1);
    }

private:
    Scalar xmin_;
    Scalar xmax_;
    Spline2d spline_;

    // Normalize x to between [0, 1] as required by Eigen interpolation
    Scalar normalize(Scalar x) const
    {
        return (x - xmin_) / (xmax_ - xmin_);
    }

    Eigen::VectorXd normalizev(const Eigen::VectorXd& v)
    {
        Eigen::VectorXd scaled(v.size());
        for (int32_t i = 0; i < v.size(); ++i) {
            scaled(i) = normalize(v(i));
        }
        return scaled;
    }

    // Creates a wide matrix of size [2, len(xs)]
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
