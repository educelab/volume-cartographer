#pragma once

#include <cassert>
#include <vector>

#include <unsupported/Eigen/Splines>

#include "vc/segmentation/lrps/Common.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class Spline
 * @brief Simple spline wrapper around Eigen::Spline
 *
 * @ingroup lrps
 */
template <typename Scalar = double, int Degree = 3>
class Spline
{
public:
    using ScalarVector = std::vector<Scalar>;
    using SplineType = Eigen::Spline<Scalar, 2>;

    Spline() = default;

    /**
     * @brief Construct a spline by fitting to a set of points
     *
     * @param xs Vector of X values
     * @param ys Vector of Y values
     */
    Spline(const ScalarVector& xs, const ScalarVector& ys) : npoints_{xs.size()}
    {
        assert(xs.size() == ys.size() && "xs and ys must be same length");
        auto points = make_wide_matrix_(xs, ys);
        spline_ = Eigen::SplineFitting<SplineType>::Interpolate(points, Degree);
    }

    /**
     * @brief %Spline evaluation at t-space value t in [0, 1]
     */
    Pixel operator()(Scalar t) const
    {
        assert(t >= 0 && t <= 1 && "t must be in range [0, 1]");
        Eigen::Vector2d val = spline_(t);
        return {val(0), val(1)};
    }

private:
    /** Number of points on the spline */
    size_t npoints_;

    SplineType spline_;

    /**
     * @brief Combine X and Y values into an npoints_ x 2 matrix
     */
    Eigen::MatrixXd make_wide_matrix_(
        const ScalarVector& xs, const ScalarVector& ys)
    {
        Eigen::MatrixXd mat{2, xs.size()};
        mat.row(0) = Eigen::VectorXd::Map(xs.data(), xs.size());
        mat.row(1) = Eigen::VectorXd::Map(ys.data(), ys.size());
        return mat;
    }
};

/**
 * A spline of degree three
 *
 * @ingroup lrps
 * */
template <typename Scalar>
using CubicSpline = Spline<Scalar, 3>;
}
}
