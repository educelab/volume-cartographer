#pragma once

/** @file */

#include <cstddef>

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
/**
 * @brief Return the size of a Linear Neighborhood calculated with the given
 * parameters
 *
 * @ingroup Util
 */
static std::size_t LinearNeighborhoodSize(
    double radius, double interval, Direction dir)
{
    // Setup Range
    radius = std::abs(radius);
    double min, max;
    switch (dir) {
        case Direction::Bidirectional: {
            min = -1 * radius;
            max = radius;
            break;
        }
        case Direction::Positive: {
            min = 0;
            max = radius;
            break;
        }
        case Direction::Negative: {
            min = -1 * radius;
            max = 0;
            break;
        }
    }

    return static_cast<std::size_t>(std::floor((max - min) / interval) + 1);
}
}  // namespace volcart