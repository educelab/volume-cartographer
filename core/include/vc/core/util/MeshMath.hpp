#pragma once

/**
 * @file MeshMath.hpp
 * @author Seth Parker
 * @date 6/7/16
 *
 * @brief Utility functions for calculations on meshes
 *
 * @ingroup Util
 */

#include "vc/core/types/ITKMesh.hpp"

namespace volcart::meshmath
{

/**
 * @brief Calculate the area of a triangle given its side lengths
 *
 * Uses a version of Heron's formula that is stable for small angles.
 */
template <
    typename T,
    std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
T TriangleArea(T a, T b, T c)
{
    // Sort the side lengths so that a >= b >= c
    auto nc = std::min(a, std::min(b, c));
    auto na = std::max(a, std::max(b, c));
    auto nb = a + b + c - na - nc;

    // Calculate the area
    auto p = (na + (nb + nc)) * (nc - (na - nb)) * (nc + (na - nb)) *
             (na + (nb - nc));

    return 0.25 * std::sqrt(p);
}

/** @brief Calculate the surface area of an ITKMesh */
double SurfaceArea(const ITKMesh::Pointer& mesh);

}  // namespace volcart::meshmath
