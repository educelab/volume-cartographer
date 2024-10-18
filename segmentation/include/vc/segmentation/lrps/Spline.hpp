#pragma once

/** @file */

#include <cassert>
#include <cstddef>
#include <vector>

#include <unsupported/Eigen/Splines>

#include "vc/segmentation/lrps/Common.hpp"

namespace volcart::segmentation
{

/**
 * @brief Combine X and Y values into an npoints x 2 matrix
 */
template <class Vector>
auto MakeWideMtx(const Vector& xs, const Vector& ys) -> Eigen::MatrixXd
{
    Eigen::MatrixXd mat{2, xs.size()};
    mat.row(0) = Eigen::VectorXd::Map(xs.data(), xs.size());
    mat.row(1) = Eigen::VectorXd::Map(ys.data(), ys.size());
    return mat;
}

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
    using Vector = std::vector<Scalar>;
    using SplineType = Eigen::Spline<Scalar, 2>;
    using Fitting = Eigen::SplineFitting<SplineType>;

    Spline() = default;

    /**
     * @brief Construct a spline by fitting to a set of points
     *
     * @param xs Vector of X values
     * @param ys Vector of Y values
     */
    Spline(const Vector& xs, const Vector& ys) : npoints_{xs.size()}
    {
        assert(xs.size() == ys.size() && "xs and ys must be same length");
        spline_ = Fitting::Interpolate(MakeWideMtx(xs, ys), Degree);
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
    std::size_t npoints_{0};
    /** Eigen spline */
    SplineType spline_;
};

/**
 * A spline of degree three
 *
 * @ingroup lrps
 * */
template <typename Scalar>
using CubicSpline = Spline<Scalar, 3>;
}  // namespace volcart::segmentation
