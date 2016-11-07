//
// Created by Hannah Hatch on 10/18/16.
//

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>
#include <tuple>

#include "boost/algorithm/string/classification.hpp"
#include "boost/algorithm/string/split.hpp"
#include "boost/filesystem/path.hpp"
#include "common/vc_defines.h"
/**
 * @class PlyReader2.h
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
class PLYReader2{
public:
    /**
     * @brief Constructs a reader with no parameters set
     *
     * This function creates a reader but does not set the input file
     * or the output mesh. This must be done before calling read.
     */
    PLYReader2();

    /**
     * @brief Constructs a reader and sets the parameters
     *
     * This function creates a reader and doesn't set the input file or
     * output mesh. These can be changed by calling the appropriate function
     * if you are reading in multiple files.
     *
     * @param path Path to the file you want to read in, relative or absolute
     * @param mesh Pointer to an ITK mesh where the data read in will be stored
     */
    PLYReader2(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh);

    /**
     * @brief Sets the input file
     * @param path Path to the file you want to read in, relative or absolute
     */
    void setPath(boost::filesystem::path path) {_inputPath = path;}
    /**
     * @brief Sets the output mesh
     * @param mesh Pointer to an ITK mesh where the data read in will be stored
     */
    void setMesh(ITK::Pointer mesh) { _outMesh = mesh; }

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
     * @struct Point
     * @brief Used to store the information for each point
     *
     * This function stores all of the information that we can
     * parse for a point. There is one of these for each point
     * that is created as points are read in.
     */
    struct Point{
        double x = 0;
        double y = 0;
        double z = 0;
        double nx = -1;
        double ny = -1;
        double nz = -1;
        std::string r = "0";
        std::string g = "0";
        std::string b = "0";
    };
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
     * List of faces in the mesh, tuple contains three point id's which
     * make up the corners of the face.
     */
    std::vector<std::tuple<int,int,int>> _faceList;
    /**
     * @brief Reads in the faces and creates a list
     */
    void _readFaces();

    /**
     * List of points in the mesh
     * @see Point
     */
    std::vector<Point> _pointList;
    /**
     * @brief Reads in each point and stores them as a Point
     * @see Point
     */
    void _readPoints();
    /** Number of vertices in the mesh */
    int _numOfVertices;
    /** Number of faces in the mesh */
    int _numOfFaces;
    /** Map that maps positions in a string to a particular attribute*/
    std::map<std::string,int> properties;
    /** Bool to keep track if the list of faces comes before the list of
     * vertices*/
    bool _facesFirst;
    /** Bool to keep track if there is a leading character in front of each face
     * line
     * telling how many vertices are in that face*/
    bool _leadingChar = true;
}; //PLYReader
} //io
} //volcart
