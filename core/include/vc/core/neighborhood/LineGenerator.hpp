#pragma once

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"

namespace volcart
{

/**
 * @class LineGenerator
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
    LineGenerator() : NeighborhoodGenerator(1) {}

    /** @overload LineGenerator() */
    static Pointer New() { return std::make_shared<LineGenerator>(); }
    /**@}*/

    /**@{*/
    Neighborhood::Extent extents() const override;
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
        Volume::Pointer v, cv::Vec3d pt, std::vector<cv::Vec3d> axes) override;
    /**@}*/
};

}  // namespace volcart
