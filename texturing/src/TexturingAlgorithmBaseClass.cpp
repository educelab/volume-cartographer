//
// Created by Seth Parker on 5/16/17.
//

#include "vc/texturing/TexturingAlgorithmBaseClass.hpp"

using namespace volcart::texturing;

size_t TexturingAlgorithmBaseClass::neighborhood_count_()
{
    auto radius = std::abs(radius_);

    // Setup Range
    double min, max;
    switch (direction_) {
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

    return static_cast<size_t>(std::floor((max - min) / interval_) + 1);
}