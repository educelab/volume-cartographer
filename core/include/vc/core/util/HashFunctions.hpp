#pragma once

/** @file */

#include <cstddef>

#include <opencv2/core.hpp>

namespace volcart
{

/**
 * Hash for 3D vector types. Must support []-operator. It's designed for integer
 * types, so don't expect much if using with floating-point types.
 *
 * Uses the hash function from:
 * https://dmauro.com/post/77011214305/a-hashing-function-for-x-y-z-coordinates
 */
template <class Vector>
struct Vec3Hash {
    std::size_t operator()(const Vector& v) const
    {
        auto max = std::max({v[0], v[1], v[2]});
        std::size_t hash = (max * max * max) + (2 * max * v[2]) + v[2];
        if (max == v[2]) {
            auto val = std::max({v[0], v[1]});
            hash += val * val;
        }
        if (v[1] >= v[0]) {
            hash += v[0] + v[1];
        } else {
            hash += v[1];
        }
        return hash;
    }
};

/** Hash for cv::Vec3i */
using Vec3iHash = Vec3Hash<cv::Vec3i>;

}  // namespace volcart
