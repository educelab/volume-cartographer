#pragma once

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Rendering.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart
{
namespace io
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
    OBJWriter(boost::filesystem::path outputPath, ITKMesh::Pointer mesh)
        : outputPath_{std::move(outputPath)}, mesh_{mesh}
    {
    }

    /** @brief Constructor with output path and textured mesh information */
    OBJWriter(
        boost::filesystem::path outputPath,
        ITKMesh::Pointer mesh,
        volcart::UVMap uvMap,
        cv::Mat uvImg)
        : outputPath_{std::move(outputPath)}
        , mesh_{mesh}
        , textCoords_{std::move(uvMap)}
        , texture_{std::move(uvImg)}
    {
    }
    /**@}*/

    /**@{*/
    /** @brief Set the output path
     *
     * write() and validate() will fail if path does not have an expected
     * file extension (.obj/.OBJ).
     */
    void setPath(const boost::filesystem::path& path) { outputPath_ = path; }

    /** @brief Set the input mesh */
    void setMesh(const ITKMesh::Pointer& mesh) { mesh_ = mesh; }

    /** @brief Set the input UV Map */
    void setUVMap(const volcart::UVMap& uvMap) { textCoords_ = uvMap; }

    /** @brief Set the input texture image */
    void setTexture(const cv::Mat& uvImg) { texture_ = uvImg; }

    /** @brief Validate parameters */
    bool validate();
    /**@}*/

    /**@{*/
    /** @brief Write the OBJ to disk
     *
     * If UV Map is not empty, automatically writes MTL and texture image.
     */
    int write();
    /**@}*/

private:
    /** Output file path */
    boost::filesystem::path outputPath_;
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
    volcart::UVMap textCoords_;
    /** Input texture image */
    cv::Mat texture_;

    /** Write the OBJ file */
    int write_obj_();
    /** Write the MTL file */
    int write_mtl_();
    /** Write the texture file */
    int write_texture_();

    /** Write the OBJ header */
    int write_header_();
    /** Write the OBJ vertices */
    int write_vertices_();
    /** Write the OBJ texture coordinates */
    int write_texture_coordinates_();
    /** Write the OBJ faces */
    int write_faces_();
};

}  // namespace io
}  // namespace volcart
