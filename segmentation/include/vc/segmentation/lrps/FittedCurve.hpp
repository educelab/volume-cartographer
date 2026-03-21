#pragma once

/** @file */

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
public:
    /** Spline type */
    using Spline = CubicSplineMT;

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
    [[nodiscard]] auto size() const -> std::size_t;

    /** @brief Return the current list of resampled points */
    [[nodiscard]] auto points() const -> const std::vector<Voxel>&;

    /** @brief Return the spline created from the input points */
    [[nodiscard]] auto spline() const -> const Spline&;

    /** @brief Resample the curve at a given t-value in [0.0, 1.0] */
    [[nodiscard]] auto eval(double t) const -> Pixel;

    /** @brief Evenly resample the curve with the same number of points as the
     * input set */
    [[nodiscard]] auto evenlySpacePoints() -> std::vector<Voxel>;

    /**
     * @brief Resamples the curve at a t-interval of resamplePerc
     * @param resamplePerc Sampling interval, in percent of original number
     * of points
     *
     * @return List of newly resampled points on the curve
     */
    auto resample(double resamplePerc = 1.0) -> std::vector<Voxel>;

    /** @brief Sample the curve into numPoints of evenly spaced points */
    [[nodiscard]] auto sample(std::size_t numPoints) const
        -> std::vector<Voxel>;

    /** @brief Returns the voxel located at index */
    auto operator()(int index) const -> Voxel;

    /**@brief Calculate the local curvature along the spline
     * @param hstep How much to move by each time you move
     *              Default: 1 point
     */
    [[nodiscard]] auto curvature(int hstep = 1) const -> std::vector<double>;

    /**@brief Calculate the arc length of the curve  */
    [[nodiscard]] auto arclength() const -> double;

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
    Spline spline_;
};
}  // namespace volcart::segmentation
