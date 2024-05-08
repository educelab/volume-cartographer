#pragma once

/** @file */

#include <cstddef>
#include <cstdint>
#include <memory>

#include "vc/core/types/NDArray.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{

/**
 * @brief N-dimensional neighborhood class
 *
 * @ingroup Neighborhoods
 */
using Neighborhood = NDArray<std::uint16_t>;

/**
 * @brief Neighborhood directional filtering options
 */
enum class Direction {
    /** @brief Only consider data in the negative normal direction */
    Negative = -1,
    /** @brief Consider data in both the positive and negative normal */
    Bidirectional = 0,
    /** @brief Only consider data in the positive normal direction */
    Positive = 1
};

/**
 * @brief Base class for neighborhood generating classes
 *
 * @ingroup Neighborhoods
 */
class NeighborhoodGenerator
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<NeighborhoodGenerator>;

    /**@{*/
    /** @brief Get the dimensionality of the neighborhood generator */
    std::size_t dim() { return dim_; }

    /** @brief Get the size of the neighborhood returned by this class
     *
     * The result of this function may vary based on the value of this class's
     * parameters
     */
    virtual Neighborhood::Extent extents() const = 0;
    /**@}*/

    /**@{*/
    /** @brief Set the sampling search radius by axis */
    void setSamplingRadius(double r, std::size_t axis = 0)
    {
        radius_[axis] = r;
    }

    /** @brief Set the sampling search radius for all axes */
    void setSamplingRadius(double r0, double r1, double r2)
    {
        radius_ = {r0, r1, r2};
    }

    /** @overload setSamplingRadius(double, double, double) */
    void setSamplingRadius(const cv::Vec3d& radii) { radius_ = radii; }

    /**
     * @brief Set the sampling interval: how frequently along the radius (in
     * Volume units) the samples are taken
     *
     * Default = 1.0
     */
    void setSamplingInterval(double i) { interval_ = i; }

    /**
     * @brief Set the filtering search direction
     *
     * Default: Bidirectional
     */
    void setSamplingDirection(Direction d) { direction_ = d; }

    /**
     * @brief Enable/Disable auto-generation of missing axes
     *
     * Derived classes are not guaranteed to make use of this functionality
     */
    void setAutoGenAxes(bool b) { autoGenAxes_ = b; }
    /**@}*/

    /**@{*/
    /** @brief Compute a neighborhood centered on a point */
    virtual Neighborhood compute(
        const Volume::Pointer& v,
        const cv::Vec3d& pt,
        const std::vector<cv::Vec3d>& axes) = 0;
    /**@}*/

protected:
    /** Default constructor */
    explicit NeighborhoodGenerator(std::size_t dim) : dim_{dim} {}

    virtual ~NeighborhoodGenerator() = default;

    /** Dimensionality of the generator */
    const std::size_t dim_{0};

    /** Radius of calculation */
    cv::Vec3d radius_{1.0, 1.0, 1.0};

    /** Sampling interval */
    double interval_{1.0};

    /** Filtering direction */
    Direction direction_{Direction::Bidirectional};

    /** Auto-generate Axes flag */
    bool autoGenAxes_{true};
};

}  // namespace volcart
