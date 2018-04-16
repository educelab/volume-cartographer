//
// Created by Seth Parker on 5/16/17.
//

#include "vc/texturing/TexturingAlgorithmBaseClass.hpp"

#include "vc/core/util/LinearNeighborhoodSize.hpp"

using namespace volcart::texturing;

size_t TexturingAlgorithmBaseClass::neighborhood_count_()
{
    return LinearNeighborhoodSize(radius_, interval_, direction_);
}