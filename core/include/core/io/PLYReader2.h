#pragma once

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <tuple>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem/path.hpp>

#include "core/types/Exceptions.h"
#include "core/vc_defines.h"

/**
 * @class PLYReader2.h
 * @author Hannah Hatch
 * @date 10/18/16
 * @brief Reads in a PLY file
 *
 * Reads in a PLY file by first reading in the header and then
 * reads in ther vertices and faces.
 *
 * It also only works for triangular meshes.
 *
 * @warning If faces aren't set, it will not hard fail but will give an error
 *
 * @ingroup Common
 */
namespace volcart
{
namespace io
{
class PLYReader2
{
public:
    /**
     * @brief Constructs a reader with no parameters set
     *
     * This function creates a reader but does not set the input file.
     * This must be done before calling read.
     */
    PLYReader2() {}

    /**
     * @brief Constructs a reader and sets the parameters
     *
     * This function creates a reader and and sets the input file.
     *  These can be changed by calling the appropriate function.
     * if you are reading in multiple files.
     *
     * @param path Path to the file you want to read in, relative or absolute
     * @param mesh Pointer to an ITK mesh where the data read in will be stored
     */
    PLYReader2(boost::filesystem::path path) : _inputPath(path) {}

    /**
     * @brief Sets the input file
     * @param path Path to the file you want to read in, relative or absolute
     */
    void setPath(boost::filesystem::path path) { _inputPath = path; }
    /**
     * @brief Sets the output mesh
     * @param mesh Pointer to an ITK mesh where the data read in will be stored
     */
    ITKMesh::Pointer getMesh() { return _outMesh; }

    /**
     * @brief Performs the actual reading in of the file
     *
     * This function simply calls the functions that read in
     * the PLY file in the appropriate order so that the vertices
     * and faces are read in the correct order.
     *
     * @warning This function assumes all parameters have been set
     */
    bool read();

private:
    /**
     * Path to the file you want to read in, realtive or absolute
     */
    boost::filesystem::path _inputPath;
    /**
     * Pointer to the mesh where the data that is read in will be stored
     */
    ITKMesh::Pointer _outMesh;

    /**
     * Stream that is used to read in the file
     */
    std::ifstream _plyFile;
    /**
     * String that represents the line currenly being parsed
     */
    std::string _line;

    /**
  * List of faces in the mesh, each cell contains
  * 3 vertices that make up that cell
  * @see common/include/common/vc_defines.h
  */
    std::vector<volcart::Cell> _faceList;

    /**
    * List of points in the mesh
    * @see common/include/common/vc_defines.h
    */
    std::vector<volcart::Vertex> _pointList;

    /** List of elements that were parsed*/
    std::vector<std::string> _elementsList;

    /** List of lines to skip for each unchecked element*/
    std::vector<int> _skippedLine;

    /** Number of vertices in the mesh */
    int _numOfVertices;

    /** Number of faces in the mesh */
    int _numOfFaces;

    /** Map that maps positions in a string to a particular attribute*/
    std::map<std::string, int> _properties;

    /** Bool to keep track if the list of faces comes before the list of
     * vertices*/
    bool _facesFirst;

    /** Bool to keep track if there is a leading character in front of each face
     * line telling how many vertices are in that face*/
    bool _leadingChar = true;

    /** Bool to keep track of if there are point normals*/
    bool _pointNorm = false;

    /**
     * @brief Creates a mesh based on stored points and faces
     *
     * This function creates a mesh using the points and faces
     * stored in the appropriate vectors that were read in.
     */
    void _createMesh();
    /**
     * @brief Parses the header
     * This function parses the header so that the location of
     * each element in a line is known. This information is stored
     * in a map and used when reading in faces and vertices.
     */
    void _parseHeader();

    /**
     * @brief Reads in the faces and creates a list
     */
    void _readFaces();

    /**
     * @brief Reads in each point and stores them as a vertex
     * @see common/include/common/vc_defines.h
     */
    void _readPoints();
};  // PLYReader
}  // io
}  // volcart
