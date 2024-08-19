#pragma once

/** @file */

#include <cassert>
#include <cstddef>
#include <vector>

#include "vc/segmentation/lrps/Common.hpp"
#include "vc/segmentation/lrps/CubicSplineMT.hpp"

namespace volcart::segmentation
{
/**
 * @class FittedCurve
 * @brief Fits a curve to a set of points for resampling
 * @ingroup lrps
 */
class FittedCurve
{
private:
    /** Number of points in the curve*/
    std::size_t npoints_{0};
    /** z-position of the curve */
    int zIndex_{0};
    /** Parameterized nodes */
    std::vector<double> ts_;
    /** List of sampled points */
    std::vector<Voxel> points_;
    /** Spline representation of curve */
    CubicSplineMT spline_;

public:
    /** @name Constructors */
    /**@{*/
    FittedCurve() = default;

    /**
     * @brief Construct curve from set of points and z-Index
     *
     * All points in generated curve are assumed to be at z = zIndex.
     *
     * @param vs List of 2D points to fit
     * @param zIndex Current location in curve
     */
    FittedCurve(const std::vector<Voxel>& vs, int zIndex);
    /**@}*/

    /** @brief Return the current number of resampled points in the spline */
    std::size_t size() const { return npoints_; }

    /** @brief Return the current list of resampled points */
    const std::vector<Voxel>& points() const { return points_; }

    /** @brief Return the spline created from the input points */
    const decltype(spline_)& spline() const { return spline_; }

    /** @brief Resample the curve at a given t-value in [0.0, 1.0] */
    Pixel eval(double t) const { 
        return spline_(t); 
        }

    /** @brief Evenly resample the curve with the same number of points as the
     * input set */
    std::vector<Voxel> evenlySpacePoints() { return resample(1.0); }

    /** @brief Resample the curve at a t-interval of resamplePerc
     *  @param resamplePerc Sampling interval, in percent of original number
     *  of points
     */
    std::vector<Voxel> resample(double resamplePerc = 1.0);

    /** @brief Resample the curve to have numPoints of evenly spaced points */
    std::vector<Voxel> sample(std::size_t numPoints) const;

    /** @brief Returns the voxel located at index */
    Voxel operator()(int index) const;

    /**@brief Calculate the local curvature along the spline
     * @param hstep How much to move by each time you move
     *              Default: 1 point
     */
    std::vector<double> curvature(int hstep = 1) const;

    /**@brief Calculate the arc length of the curve  */
    double arclength() const;
};
}  // namespace volcart::segmentation
