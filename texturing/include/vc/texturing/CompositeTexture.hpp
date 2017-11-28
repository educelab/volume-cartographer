#pragma once

#include "vc/texturing/TexturingAlgorithmBaseClass.hpp"

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
class CompositeTexture : public TexturingAlgorithmBaseClass
{
public:
    /** Default destructor */
    ~CompositeTexture() = default;

    /** Filter list */
    enum class Filter { Minimum, Maximum, Median, Mean, MedianAverage };

    /**@{*/
    /**
     * @brief Set the filtering method
     *
     * Default: Maximum
     */
    void setFilter(Filter f) { filter_ = f; }
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute() override;
    /**@}*/

private:
    /** Filter method */
    Filter filter_{Filter::Maximum};

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
