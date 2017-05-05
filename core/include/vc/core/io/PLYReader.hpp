#pragma once

#include <fstream>

#include <boost/filesystem/path.hpp>

#include "vc/core/vc_defines.hpp"

namespace volcart
{
namespace io
{

/**
 * @class PLYReader
 * @author Hannah Hatch
 * @date 10/18/16
 *
 * @brief Read a PLY file to an ITKMesh
 *
 * Only supports vertices, vertex normals, and faces. Will write a warning to
 * std::cerr if the PLY file doesn't contain faces.
 *
 * @ingroup IO
 */
class PLYReader
{
public:
    /**@{*/
    /** @brief Default constructor */
    PLYReader() = default;

    /** @brief Constructor with input file path */
    explicit PLYReader(boost::filesystem::path path)
        : inputPath_{std::move(path)}
    {
    }
    /**@}*/

    /**@{*/
    /** @brief Set the input file path */
    void setPath(const boost::filesystem::path& path) { inputPath_ = path; }

    /** @brief Get the parsed input as an ITKMesh */
    ITKMesh::Pointer getMesh() { return outMesh_; }
    /**@}*/

    /**@{*/
    /** @brief Parse the input file */
    ITKMesh::Pointer read();
    /**@}*/

private:
    /** Input file path */
    boost::filesystem::path inputPath_;
    /** Input file stream */
    std::ifstream plyFile_;
    /** Output mesh */
    ITKMesh::Pointer outMesh_;
    /** Line currently being parsed */
    std::string line_;
    /** Temporary face list */
    std::vector<volcart::Cell> faceList_;
    /** Temporary vertex list */
    std::vector<volcart::Vertex> pointList_;
    /** List of elements parsed from the header */
    std::vector<std::string> elementsList_;
    /** Number of vertices expected in the parsed mesh */
    int numVertices_;
    /** Number of faces expected in the parsed mesh */
    int numFaces_;
    /** Number of lines used by the elements we can't handle */
    std::vector<int> skippedLine_;
    /** Maps positions in a string to a particular attribute */
    std::map<std::string, int> properties_;

    /**
     * Tracks if there is a leading character in front of each face line. This
     * character tells how many vertices are in that face.
     */
    bool hasLeadingChar_ = true;

    /** Track if there are vertex normals */
    bool hasPointNorm_ = false;

    /** @brief Construct outMesh_ from the temporary vertices and faces */
    void create_mesh_();

    /**
     * @brief Parse the PLY header
     *
     * The header defines the expected layout for the body of the PLY file,
     * including the count for each element (e.g. vertices, faces, etc). This
     * information is stored in elementsList_ and used when reading in faces
     * and vertices.
     */
    void parse_header_();

    /** @brief Fill the temporary face list with parsed face information */
    void read_faces_();

    /** @brief Fill the temporary vertex list with parsed vertex information */
    void read_points_();
};
}  // io
}  // volcart
