#pragma once

#include <cassert>
#include <unsupported/Eigen/Splines>
#include <vector>

#include "segmentation/lrps/Common.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class Spline
 * @brief Used to create Splines to generate the fitted curve
 *
 * @ingroup lrps
 */
template <typename Scalar = double, int Degree = 3>
class Spline
{
public:
    /** Sets up a vector of pairs of doubles and 3 */
    using ScalarVector = std::vector<Scalar>;
    /** Sets the Spline to be Quadratic */
    using SplineType = Eigen::Spline<Scalar, 2>;

    /**
     * @brief Sets the default constructor
     */
    Spline() = default;

    /**
     * @brief Constructor which sets the two scalar vectors
     *
     * Uses two scalar vectors that are the same size to create a matrix
     * of points and then interpolate them onto a cubic spline
     *
     * @param xs First Scalar Vector
     * @param ys Second Scalar Vector
     */
    Spline(const ScalarVector& xs, const ScalarVector& ys) : npoints_{xs.size()}
    {
        assert(xs.size() == ys.size() && "xs and ys must be same length");
        auto points = make_wide_matrix_(xs, ys);
        spline_ = Eigen::SplineFitting<SplineType>::Interpolate(points, Degree);
    }

    /**
     * @brief Plane evaluation at t-space value t in [0, 1]
     */
    Pixel operator()(Scalar t) const
    {
        assert(t >= 0 && t <= 1 && "t must be in range [0, 1]");
        Eigen::Vector2d val = spline_(t);
        return {val(0), val(1)};
    }

private:
    /** Number of points on the spline*/
    size_t npoints_;
    /** Quadratic spline that is generated*/
    SplineType spline_;

    /**
     * @brief Creates a matrix using 2 Scalar vectors
     * @param xs Vector to make up the first row
     * @param ys Vector to make up the second row
     * @return Generated matrix
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

template <typename Scalar>
/** A spline of degree three using the previous definition of Scalar*/
using CubicSpline = Spline<Scalar, 3>;
}
}
