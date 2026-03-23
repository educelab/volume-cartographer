#pragma once

/** @file */

#include <vector>

#include "vc/segmentation/lrps/Common.hpp"

namespace volcart::segmentation
{

/**
 * @brief Cubic spline
 *
 * @author    Julian Schilliger
 * @date      September 2023
 *
 * @details Cubic spline class which uses multiple threads (via OpenMP, when
 * available) to fit to the provided knots.
 */
class CubicSpline
{
public:
    /** Default constructor */
    CubicSpline() = default;
    /** Construct and fit to separated x, y knot pairs */
    CubicSpline(const std::vector<double>& x, const std::vector<double>& y);
    /** Construct and fit to a set of knots */
    explicit CubicSpline(const std::vector<Voxel>& vs);
    /** Default destructor */
    ~CubicSpline() = default;

    /** Copy constructor */
    CubicSpline(const CubicSpline&) = default;

    /** Copy assignment operator */
    auto operator=(const CubicSpline&) -> CubicSpline& = default;

    /**
     * @brief %Spline evaluation at t-space value t in [0, 1]
     */
    auto operator()(double t) const -> Pixel;

private:
    /** x params */
    std::vector<double> aX_, bX_, cX_, dX_;
    /** y params */
    std::vector<double> aY_, bY_, cY_, dY_;
    /** Percent position of knots in total number of knots */
    std::vector<double> rangeXY_;
    /** Lengths of subsegments */
    std::vector<double> subsegLens_;
    /** Cumulative lengths of subsegments */
    std::vector<double> cumuLens_;
};

}  // namespace volcart::segmentation
