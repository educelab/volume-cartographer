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
 * @file PlyReader2.h
 * @author Hannah Hatch
 * @date 10/18/16
 * @brief Reads in a PLY file
 *
 * Reads in a PLY file by reading in the vertices and then the faces. It can
 * only parse point normals, points, and faces. It ignores any other features.
 *
 * It also only works for triangular meshes.
 *
 * @warning If faces aren't set, it will not hard fail but will give an error
 *
 * @ingroup VolCore
 *
 * @param path The location of the file you want to read in
 * @param mesh Where you want to store the mesh that's been read in
 */
namespace volcart
{
namespace io
{
class PLYReader2{
public:
    PLYReader2();
    PLYReader2(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh);

    void setPath(boost::filesystem::path path) {_inputPath = path;}


    bool read();


private:
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
    boost::filesystem::path _inputPath;
    ITKMesh::Pointer _outMesh;

    std::ifstream _plyFile;
    std::string _line;

    void _createMesh();
    void _parseHeader();


    std::vector<std::tuple<int,int,int>> _faceList;
    void _readFaces();

    std::vector<Point> _pointList;
    void _readPoints();
    int _numOfVertices;
    int _numOfFaces;
    std::map<std::string,int> properties;

    bool _facesFirst;
    bool _leadingChar = true;
}; //PLYReader
} //io
} //volcart
