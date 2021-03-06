#pragma once

/** @file */

#include <fstream>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Texture.hpp"

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
    PLYWriter(
        volcart::filesystem::path outputPath, const ITKMesh::Pointer& mesh)
        : outputPath_{std::move(outputPath)}, mesh_{mesh}
    {
    }
    /** @brief Constructor with output path and textured mesh information */
    PLYWriter(
        volcart::filesystem::path outputPath,
        const ITKMesh::Pointer& mesh,
        volcart::Texture texture)
        : outputPath_{std::move(outputPath)}
        , mesh_{mesh}
        , texture_{std::move(texture)}
    {
    }
    /**@}*/

    /**@{*/
    /** @brief Set the output path
     *
     * write() and validate() will fail if path does not have an expected
     * file extension (.ply/.PLY).
     */
    void setPath(const volcart::filesystem::path& path) { outputPath_ = path; }

    /** @brief Return the output path */
    volcart::filesystem::path getPath() const { return outputPath_; }

    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& mesh) { mesh_ = mesh; }

    /** @brief Set texture information from a volcart::Texture */
    void setTexture(const volcart::Texture& texture) { texture_ = texture; }

    /** @brief Set per-vertex color information */
    void setVertexColors(const std::vector<uint16_t>& c);
    /**@}*/

    /**@{*/
    /** @brief Write the PLY to disk
     *
     * If UV Map is not empty, automatically writes per-vertex texture
     * information.
     */
    int write();
    /**@}*/

private:
    /** Output file path */
    volcart::filesystem::path outputPath_;
    /** Output file stream */
    std::ofstream outputMesh_;
    /** Input mesh */
    ITKMesh::Pointer mesh_;
    /** Input texture information */
    volcart::Texture texture_;
    /** Vertex colors */
    std::vector<uint16_t> vcolors_;

    /** @brief Write the PLY header */
    int write_header_();

    /**
     * @brief Write the PLY vertices
     *
     * Lines are formatted: \n
     * `x y z nx ny nz`
     */
    int write_vertices_();
    /**@brief Write the PLY faces
     *
     * Lines are formatted: \n
     * `[n vertices in face] v1 v2 ... vn`
     */
    int write_faces_();
};
}  // namespace volcart::io
