#pragma once

/** @file */

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"

namespace volcart
{

/**
 * @brief Class for generating line-like neighborhoods from a point in a Volume
 *
 * @ingroup Neighborhoods
 */
class LineGenerator : public NeighborhoodGenerator
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<LineGenerator>;

    /**@{*/
    /** @brief Default Constructor */
    LineGenerator();

    /** @overload LineGenerator() */
    static Pointer New();
    /**@}*/

    /**@{*/
    [[nodiscard]] auto extents() const -> Neighborhood::Extent override;

    /**
     * @brief Returns the list of scalar offsets along the neighborhood axis
     *
     * Each offset is a scalar distance from the origin point. The offsets
     * are computed from the generator's radius, interval, and direction
     * settings and correspond 1-to-1 with the samples returned by compute().
     */
    [[nodiscard]] auto offsets() const -> std::vector<double>;
    /**@}*/

    /**@{*/
    /**
     * @brief @copybrief NeighborhoodGenerator::compute()
     *
     * This method computes a line-like neighborhood, centered on a
     * point embedded in a Volume. The orientation of the neighborhood is
     * determined by the first provided axis. At least one axis vector is
     * required.
     *
     * This class does not make use of the value of `setAutoGenAxes()`.
     */
    Neighborhood compute(
        const Volume::Pointer& v,
        const cv::Vec3d& pt,
        const std::vector<cv::Vec3d>& axes) override;
    /**@}*/
};

}  // namespace volcart
