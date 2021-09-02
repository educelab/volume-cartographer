#pragma once

/** @file */

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
/**
 * @brief Convenience type for textured meshes
 *
 * @ingroup Types
 */
struct TexturedMesh {
    /** @brief Default constructor */
    TexturedMesh() = default;
    /** @brief Constructor with member components */
    TexturedMesh(ITKMesh::Pointer m, UVMap::Pointer u, cv::Mat i)
        : mesh{std::move(m)}, uv{std::move(u)}, img{std::move(i)}
    {
    }
    /** @brief Mesh geometry */
    ITKMesh::Pointer mesh;
    /** @brief UV map */
    UVMap::Pointer uv;
    /** @brief Texture image */
    cv::Mat img;
};
}  // namespace volcart