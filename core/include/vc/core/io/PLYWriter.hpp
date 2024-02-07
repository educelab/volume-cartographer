#pragma once

/** @file */

#include <cstdint>
#include <fstream>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart::io
{
/**
 * @class PLYWriter
 * @author Seth Parker
 * @date 10/30/15
 *
 * @brief Write an ITKMesh to a PLY file
 *
 * Writes both textured and untextured meshes in ASCII PLY format. Texture
 * information is automatically written if the volcart::Texture has images
 * and if the UV map is set and is not empty.
 *
 * Assumes that vertices have vertex normal information.
 *
 * @ingroup IO
 */
class PLYWriter
{
public:
    /**@{*/
    /** @brief Default constructor */
    PLYWriter() = default;

    /** @brief Constructor with output path and input mesh */
    PLYWriter(filesystem::path outputPath, ITKMesh::Pointer mesh);

    /** @brief Constructor with output path and textured mesh information */
    PLYWriter(
        filesystem::path outputPath, ITKMesh::Pointer mesh, cv::Mat texture);
    /**@}*/

    /**@{*/
    /** @brief Set the output path
     *
     * write() and validate() will fail if path does not have an expected
     * file extension (.ply/.PLY).
     */
    void setPath(const filesystem::path& path);

    /** @brief Set the input mesh */
    void setMesh(ITKMesh::Pointer mesh);

    /** @brief Set the input UV Map */
    void setUVMap(UVMap::Pointer uvMap);

    /** @brief Set texture image */
    void setTexture(cv::Mat texture);

    /** @brief Set per-vertex color information */
    void setVertexColors(const std::vector<std::uint16_t>& c);
    /**@}*/

    /**@{*/
    /**
     * @brief Write the PLY to disk
     *
     * If UV Map is not empty, automatically writes per-vertex texture
     * information.
     */
    auto write() -> int;
    /**@}*/

private:
    /** Output file path */
    filesystem::path outputPath_;
    /** Output file stream */
    std::ofstream outputMesh_;
    /** Input mesh */
    ITKMesh::Pointer mesh_;
    /** Input UV map */
    UVMap::Pointer uvMap_;
    /** Input texture image */
    cv::Mat texture_;
    /** Vertex colors */
    std::vector<std::uint16_t> vcolors_;

    /** @brief Write the PLY header */
    auto write_header_() -> int;

    /**
     * @brief Write the PLY vertices
     *
     * Lines are formatted:
     *
     * `x y z nx ny nz`
     */
    auto write_vertices_() -> int;
    /**@brief Write the PLY faces
     *
     * Lines are formatted:
     *
     * `[n vertices in face] v1 v2 ... vn`
     */
    auto write_faces_() -> int;
};
}  // namespace volcart::io
