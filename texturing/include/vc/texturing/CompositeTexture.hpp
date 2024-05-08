#pragma once

/** @file */

#include "vc/texturing/TexturingAlgorithm.hpp"

namespace volcart::texturing
{

/**
 * @brief Generate a texture image using a variety of composite volume filters
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
class CompositeTexture : public TexturingAlgorithm
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<CompositeTexture>;

    /** Make shared pointer */
    static auto New() -> Pointer;

    /** Default constructor */
    CompositeTexture() = default;
    /** Default destructor */
    ~CompositeTexture() override = default;
    /** Default copy constructor */
    CompositeTexture(CompositeTexture&) = default;
    /** Default move constructor */
    CompositeTexture(CompositeTexture&&) = default;
    /** Default copy operator */
    auto operator=(const CompositeTexture&) -> CompositeTexture& = default;
    /** Default move operator */
    auto operator=(CompositeTexture&&) -> CompositeTexture& = default;

    /** Filter list */
    enum class Filter {
        /** @brief Select the minimum intensity value */
        Minimum = 0,
        /** @brief Select the maximum intensity value */
        Maximum,
        /** @brief Select the median intensity value */
        Median,
        /** @brief Calculate the mean intensity value */
        Mean,
        /** @brief Calculate the mean of the median 70% of values */
        MedianAverage
    };

    /**@{*/
    /**
     * @brief Set the Neighborhood generator
     *
     * This class supports generators of dimension >= 1
     */
    void setGenerator(NeighborhoodGenerator::Pointer g);

    /**
     * @brief Set the filtering method
     *
     * Default: Maximum
     */
    void setFilter(Filter f);
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    auto compute() -> Texture override;
    /**@}*/

private:
    /** Neighborhood shape */
    NeighborhoodGenerator::Pointer gen_;

    /** Filter method */
    Filter filter_{Filter::Maximum};
};
}  // namespace volcart::texturing
