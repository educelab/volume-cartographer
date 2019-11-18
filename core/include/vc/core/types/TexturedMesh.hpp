#pragma once

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
    TexturedMesh(const ITKMesh::Pointer& m, UVMap u, cv::Mat i)
        : mesh{m}, uv{std::move(u)}, img{std::move(i)}
    {
    }
    /** @brief Mesh geometry */
    ITKMesh::Pointer mesh;
    /** @brief UV map */
    UVMap uv;
    /** @brief Texture image */
    cv::Mat img;
};
}  // namespace volcart