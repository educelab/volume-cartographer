#pragma once

/** @file */

#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart::io
{
/**
 * @class OBJWriter
 * @author Seth Parker
 * @date 6/24/15
 *
 * @brief Write an ITKMesh to an OBJ file
 *
 * Writes both textured and untextured meshes in ASCII OBJ format. Texture
 * information is automatically written if a UV map is set and is not empty.
 *
 * @ingroup IO
 */
class OBJWriter
{

public:
    /**@{*/
    /** @brief Default constructor */
    OBJWriter() = default;

    /** @brief Constructor with output path and input mesh */
    OBJWriter(filesystem::path outputPath, ITKMesh::Pointer mesh);

    /** @brief Constructor with output path and textured mesh information */
    OBJWriter(
        filesystem::path outputPath,
        ITKMesh::Pointer mesh,
        UVMap::Pointer uvMap,
        cv::Mat uvImg);
    /**@}*/

    /**@{*/
    /** @brief Set the output path
     *
     * write() and validate() will fail if path does not have an expected
     * file extension (.obj/.OBJ).
     */
    void setPath(const filesystem::path& path);

    /** @brief Set the input mesh */
    void setMesh(ITKMesh::Pointer mesh);

    /** @brief Set the input UV Map */
    void setUVMap(UVMap::Pointer uvMap);

    /** @brief Set the input texture image */
    void setTexture(cv::Mat uvImg);
    /**@}*/

    /**@{*/
    /** @brief Write the OBJ to disk
     *
     * If UV Map is not empty, automatically writes MTL and texture image.
     */
    auto write() -> int;
    /**@}*/

private:
    /** Output file path */
    filesystem::path outputPath_;
    /** Output OBJ filestream */
    std::ofstream outputMesh_;
    /** Output MTL filestream */
    std::ofstream outputMTL_;

    /** Keeps track of what info we have about each point in the mesh. Used for
     * building OBJ faces.
     *
     * [ Point Index, {v, vt, vn} ]
     *
     * v = vertex index number \n
     * vt = UV coordinate index number \n
     * vn = vertex normal index number \n
     */
    std::map<uint32_t, cv::Vec3i> pointLinks_;

    /** Input mesh */
    ITKMesh::Pointer mesh_;
    /** Input UV map */
    UVMap::Pointer uvMap_;
    /** Input texture image */
    cv::Mat texture_;

    /** Write the OBJ file */
    auto write_obj_() -> int;
    /** Write the MTL file */
    auto write_mtl_() -> int;
    /** Write the texture file */
    auto write_texture_() -> int;

    /** Write the OBJ header */
    auto write_header_() -> int;
    /** Write the OBJ vertices */
    auto write_vertices_() -> int;
    /** Write the OBJ texture coordinates */
    auto write_texture_coordinates_() -> int;
    /** Write the OBJ faces */
    auto write_faces_() -> int;
};

}  // namespace volcart::io
