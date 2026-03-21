#pragma once

/** @file */

#include <vector>

#include "vc/segmentation/lrps/Common.hpp"

namespace volcart::segmentation
{

/**
 * @brief Multi-threaded cubic spline
 *
 * @author    Julian Schilliger
 * @date      September 2023
 *
 * @details Cubic spline class which uses multiple threads to fit to the
 * provided knots.
 */
class CubicSplineMT
{
public:
    /** Default constructor */
    CubicSplineMT() = default;
    /** Construct and fit to separated x, y knot pairs */
    CubicSplineMT(const std::vector<double>& x, const std::vector<double>& y);
    /** Construct and fit to a set of knots */
    explicit CubicSplineMT(const std::vector<Voxel>& vs);
    /** Default destructor */
    ~CubicSplineMT() = default;

    /** Copy constructor */
    CubicSplineMT(const CubicSplineMT&) = default;

    /** Copy assignment operator */
    auto operator=(const CubicSplineMT&) -> CubicSplineMT& = default;

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