#pragma once

#include <cassert>
#include <vector>
#include "segmentation/lrps/Common.hpp"
#include "segmentation/lrps/Spline.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class FittedCurve
 * @brief Creates a curve to fit a set of points
 * @ingroup lrps
 */
class FittedCurve
{
private:
    /** Number of points in the curve*/
    size_t npoints_;
    /** Where you are in the curve */
    int zIndex_;
    /** Point normals */
    std::vector<double> ts_;
    /** List of points */
    std::vector<Voxel> points_;
    /** Creates a spline based on the points */
    CubicSpline<double> spline_;

public:
    /**
     * @brief Initializes a fitted curve with defualt settings
     */
    FittedCurve() : npoints_(0), zIndex_(0), ts_(), points_(), spline_() {}

    /**
     * @brief Initializes a fitted curve and sets the points and a place to
     * start
     * @param vs List of points where a curve is to be fit
     * @param zIndex Current location in curve
     */
    FittedCurve(const std::vector<Voxel>& vs, int zIndex);

    /** @brief Returns the number of points in the spline*/
    size_t size() const { return npoints_; }

    /** @brief Returns the list of points in the spline*/
    const std::vector<Voxel>& points() const { return points_; }

    /** @brief Returns the spline created from the points */
    const decltype(spline_) & spline() const { return spline_; }

    /** @brief Returns the value for a given t */
    Pixel eval(double t) const { return spline_(t); }

    /** @brief Returns a vector of points that are evenly spaced*/
    std::vector<Voxel> evenlySpacePoints() { return resample(1.0); }

    /** @brief Returns a vector of points that have been resapmled
     *  @param resamplePerc Percent of faces to be remaining after resample
     *                      Default: 1, no faces removed
     */
    std::vector<Voxel> resample(double resamplePerc = 1.0);

    /** @brief Returns a vector with the first numPoints points*/
    std::vector<Voxel> sample(size_t numPoints) const;

    /** @brief Returns the voxel located at index */
    Voxel operator()(int index) const;

    /**@brief Calculates the curvature of the spline
     * @param hstep How much to move by each time you move
     *              Default: 1 point
     */
    std::vector<double> curvature(int hstep = 1) const;

    /**@brief Calculates the archlenght of the curve  */
    double arclength() const;
};
}
}
