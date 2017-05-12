#pragma once

#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
namespace texturing
{

/**
 * @class CompositeTexture
 * @author Seth Parker
 * @date 05/15/2017
 *
 * @brief Generate a Texture using a variety of composite volume filters
 *
 * This class generates a texture image by filtering the Volume using one of a
 * number of composite measures:
 *
 * - Minimum: Filter a neighborhood to select the minimum intensity.
 * - Maximum: Filter a neighborhood to select the maximum intensity.
 * - Median: Filter a neighborhood to select the median intensity.
 * - Mean: Filter a neighborhood by averaging the intensities.
 * - Median + Averaging: Filter a neighborhood by averaging the median 70%.
 *
 * @ingroup Texture
 */
class CompositeTexture
{
public:
    /** Filter list */
    enum class Filter { Minimum, Maximum, Median, Mean, MedianAverage };

    /**@{*/
    /** @brief Set the input PerPixelMap */
    void setPerPixelMap(PerPixelMap ppm) { ppm_ = std::move(ppm); }

    /** @brief Set the input Volume */
    void setVolume(Volume::Pointer vol) { vol_ = std::move(vol); }

    /**
     * @brief Set the filtering method
     *
     * Default: Maximum
     */
    void setFilter(Filter f) { filter_ = f; }

    /** @brief Set the sampling search radius: the distance from the mesh to
     * consider for compositing */
    void setSamplingRadius(double r) { radius_ = r; }

    /**
     * @brief Set the sampling interval: how frequently the voxels along the
     * radius are sampled for compositing purposes
     *
     * Default = 1.0
     */
    void setSamplingInterval(double i) { interval_ = i; }

    /**
     * @brief Set the filtering search direction: which "side" of the mesh to
     * consider when compositing
     *
     * Default: Bidirectional
     */
    void setSamplingDirection(Direction d) { direction_ = d; }
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute();
    /**@}*/

    /**@{*/
    /** @brief Get the generated Texture */
    const Texture& getTexture() const { return result_; }

    /** @copydoc getTexture() const */
    Texture& getTexture() { return result_; }
    /**@}*/

private:
    /** PPM */
    PerPixelMap ppm_;
    /** Volume */
    Volume::Pointer vol_;
    /** Search radius */
    double radius_;
    /** Search direction */
    Direction direction_{Direction::Bidirectional};
    /** Search sampling interval */
    double interval_{1.0};
    /** Filter method */
    Filter filter_{Filter::Maximum};
    /** Result */
    Texture result_;

    /** Filter a neighborhood based on filter_ */
    uint16_t filter_neighborhood_(const Neighborhood& n);
    /** Return the minimum value */
    uint16_t min_(Neighborhood n);
    /** Return the maximum value */
    uint16_t max_(Neighborhood n);
    /** Return the median value */
    uint16_t median_(Neighborhood n);
    /** Return the average value */
    uint16_t mean_(Neighborhood n);
    /** Return the average of the median `range`. `range` is [0, 1] and is
     * a percent of the neighborhood. */
    uint16_t median_mean_(Neighborhood n, double range);
};
}
}
