#pragma once

/** @file */

#include "vc/texturing/FlatteningAlgorithm.hpp"

namespace volcart
{
namespace texturing
{
/**
 * @brief Computes a 2D parameterization of a triangular mesh using
 * orthographic projection
 *
 * This class performs naive mesh parameterization using orthographic
 * projection. The largest plane of the mesh bounding box is selected as the
 * projection plane. Each vertex is orthographically projected onto this plane,
 * and the UV coordinate is calculated as the relative position of the
 * projected point within the bounds of the projection plane.
 *
 * This class is primarily only useful for meshes which are already planar
 * or semi-planar.
 *
 * @ingroup UV
 */
class OrthographicProjectionFlattening : public FlatteningAlgorithm
{
public:
    /** @brief Default constructor */
    OrthographicProjectionFlattening() = default;

    /** Make a new shared instance */
    static Pointer New();

    /** @brief Compute the parameterization */
    ITKMesh::Pointer compute() override;
};

}  // namespace texturing
}  // namespace volcart