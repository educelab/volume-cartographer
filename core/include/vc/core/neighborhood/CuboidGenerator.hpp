#pragma once

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"

namespace volcart
{

/**
 * @class CuboidGenerator
 * @brief Class for generating box-like neighborhoods from a point in a Volume
 *
 * @ingroup Neighborhoods
 */
class CuboidGenerator : public NeighborhoodGenerator
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<CuboidGenerator>;

    /**@{*/
    /** @brief Default Constructor */
    CuboidGenerator() : NeighborhoodGenerator(3) {}

    /** @overload CuboidGenerator() */
    static Pointer New() { return std::make_shared<CuboidGenerator>(); }
    /**@}*/

    /**@{*/
    Neighborhood::Extent extents() const override;
    /**@}*/

    /**@{*/
    /**
     * @brief @copybrief NeighborhoodGenerator::compute()
     *
     * This method computes a cuboid (box-like) neighborhood, centered on a
     * point embedded in a Volume. The orientation of the neighborhood is
     * determined by the provided axes. At least one axis vector is required.
     * Only the first 3 provided vectors will be used.
     *
     * If `setAutoGenAxes()` is `true`, this method will automatically
     * generate any missing axes using the cross product. For example, if one
     * axis is provided, the 2nd and 3rd will be generated, but if two are
     * provided, only the 3rd will be generated.
     */
    Neighborhood compute(
        const Volume::Pointer& v,
        const cv::Vec3d& pt,
        const std::vector<cv::Vec3d>& axes) override;
    /**@}*/
};

}  // namespace volcart
