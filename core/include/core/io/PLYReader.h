#pragma once

#include <fstream>

#include <boost/filesystem/path.hpp>

#include "core/vc_defines.h"

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
 * @ingroup Core
 */
class PLYReader
{
public:
    //** @name Constructors */
    //@{
    /**
     * @brief Default constructor
     */
    PLYReader() {}

    /**
     * @brief Constructs a reader and initialize the input path parameter
     *
     * @param path Path to the file to be read
     */
    explicit PLYReader(boost::filesystem::path path)
        : inputPath_(std::move(path))
    {
    }
    //@}

    /**
     * @brief Set the input file path
     * @param path Path to the file to be read
     */
    void setPath(boost::filesystem::path path) { inputPath_ = std::move(path); }

    /**
     * @brief Get the parsed output as an ITKMesh
     */
    ITKMesh::Pointer getMesh() { return outMesh_; }

    /**
     * @brief Parse the input file
     */
    bool read();

private:
    /**
     * Path to the file to be read in, relative or absolute
     */
    boost::filesystem::path inputPath_;

    /**
     * Mesh where the parsed data will be stored
     */
    ITKMesh::Pointer outMesh_;

    /**
     * Input file stream
     */
    std::ifstream plyFile_;

    /**
     * Line currently being parsed
     */
    std::string line_;

    /**
     * Temporary face list
     */
    std::vector<volcart::Cell> faceList_;

    /**
    * Temporary vertex list
    */
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

    /** Track if there is a leading character in front of each face line telling
     * how many vertices are in that face */
    bool hasLeadingChar_ = true;

    /** Track if there are point normals */
    bool hasPointNorm_ = false;

    /**
     * @brief Generate a mesh from the temporary vertices and faces
     */
    void create_mesh_();
    /**
     * @brief Parse the header
     *
     * The header defines the expected layout for the body of the PLY file,
     * including the count for each element. This information is stored
     * in a map and used when reading in faces and vertices.
     */
    void parse_header_();

    /**
     * @brief Fill the temporary face list with parsed face information
     */
    void read_faces_();

    /**
     * @brief Fill the temporary vertex list with parsed vertex information
     */
    void read_points_();
};  // PLYReader
}  // io
}  // volcart
