/**@file PlyReader2.cpp */

#include "common/io/PlyReader2.h"

using Props = std::pair<char, int>;

namespace volcart
{
namespace io
{

bool PLYReader2::read()
{
    _plyFile.open(_inputPath.string());
    if (!_plyFile.is_open()) {
        auto msg = "Open file " + _inputPath.string() + " failed.";
        throw volcart::IOException(msg);
    }
    _parseHeader();
    if (_facesFirst) {
        _readFaces();
        _readPoints();
    } else {
        _readPoints();
        _readFaces();
    }
    _plyFile.close();
    _createMesh();
    return true;
}

void PLYReader2::_parseHeader()
{
    std::vector<std::string> facenum;
    std::vector<std::string> vertnum;
    std::getline(_plyFile, _line);
    while (_line.find("element", 0) == std::string::npos) {
        std::getline(_plyFile, _line);
    }

    // Assumes that if there's face information, there's also vertex information
    if (_line.find("vertex", 0) == std::string::npos &&
        _line.find("face", 0) != std::string::npos) {
        _facesFirst = true;
        boost::split(
            facenum, _line, boost::is_any_of(" "), boost::token_compress_on);
        _numOfFaces = std::stoi(facenum[2]);
        std::getline(_plyFile, _line);

        if (_line.find("uchar") == std::string::npos) {
            _leadingChar = false;
        }

        std::getline(_plyFile, _line);
        boost::split(
            vertnum, _line, boost::is_any_of(" "), boost::token_compress_on);
        _numOfVertices = std::stoi(vertnum[2]);
        std::getline(_plyFile, _line);
        int currentLine = 0;

        while (_line.find("element", 0) == std::string::npos &&
               _line.find("end_header", 0) == std::string::npos) {
            std::vector<std::string> curline;
            boost::split(
                curline, _line, boost::is_any_of(" "),
                boost::token_compress_on);
            if (curline[2] == "nx") {
                _pointNorm = true;
            }
            properties[curline[2]] = currentLine;
            std::getline(_plyFile, _line);
            currentLine++;
        }
    }

    else if (
        _line.find("vertex", 0) == std::string::npos &&
        _line.find("face", 0) == std::string::npos) {
        auto msg = "No header information, file cannot be parsed";
        throw volcart::IOException(msg);
    }

    else {
        _facesFirst = false;
        boost::split(
            vertnum, _line, boost::is_any_of(" "), boost::token_compress_on);
        _numOfVertices = std::stoi(vertnum[2]);
        std::getline(_plyFile, _line);
        int currentLine = 0;

        while (_line.find("element", 0) == std::string::npos &&
               _line.find("end_header", 0) == std::string::npos) {
            std::vector<std::string> curline;
            boost::split(
                curline, _line, boost::is_any_of(" "),
                boost::token_compress_on);
            if (curline[2] == "nx") {
                _pointNorm = true;
            }
            properties[curline[2]] = currentLine;
            std::getline(_plyFile, _line);
            currentLine++;
        }

        boost::split(
            facenum, _line, boost::is_any_of(" "), boost::token_compress_on);

        if (_line.find("face", 0) == std::string::npos) {
            std::cerr << "Warning: No face information found, reading in "
                         "vertices only"
                      << std::endl;
            _numOfFaces = 0;
        } else {
            _numOfFaces = std::stoi(facenum[2]);
            std::getline(_plyFile, _line);
            if (_line.find("uchar") == std::string::npos) {
                _leadingChar = false;
            }
        }

        std::getline(_plyFile, _line);
    }

    while (_line.find("end_header", 0) == std::string::npos) {
        std::getline(_plyFile, _line);
    }
    std::getline(_plyFile, _line);

}  // ParseHeader

void PLYReader2::_readPoints()
{
    int i;
    for (i = 0; i < _numOfVertices; i++) {
        volcart::Vertex curPoint;
        std::vector<std::string> curLine;
        boost::split(
            curLine, _line, boost::is_any_of(" "), boost::token_compress_on);
        curPoint.x = std::stod(curLine[properties["x"]]);
        curPoint.y = std::stod(curLine[properties["y"]]);
        curPoint.z = std::stod(curLine[properties["z"]]);
        if (properties.find("nx") != properties.end()) {
            curPoint.nx = std::stod(curLine[properties["nx"]]);
            curPoint.ny = std::stod(curLine[properties["ny"]]);
            curPoint.nz = std::stod(curLine[properties["nz"]]);
        }
        if (properties.find("r") != properties.end()) {
            curPoint.r = stoi(curLine[properties["r"]]);
            curPoint.g = stoi(curLine[properties["g"]]);
            curPoint.b = stoi(curLine[properties["b"]]);
        }
        _pointList.push_back(curPoint);
        std::getline(_plyFile, _line);
    }
}

void PLYReader2::_readFaces()
{
    int i;
    for (i = 0; i < _numOfFaces; i++) {
        std::vector<std::string> curFace;
        volcart::Cell face;
        boost::split(
            curFace, _line, boost::is_any_of(" "), boost::token_compress_on);
        if (_leadingChar) {
            int points_per_face = std::stoi(curFace[0]);
            if (points_per_face != 3) {
                auto msg = "Error: Not a Triangular Mesh";
                throw volcart::IOException(msg);
            } else {
                face = Cell(
                    std::stoul(curFace[1]), std::stoul(curFace[2]),
                    std::stoul(curFace[3]));
                _faceList.push_back(face);
            }

        } else {
            if (curFace.size() != 3) {
                auto msg = "Error: Not a Triangular Mesh";
                throw volcart::IOException(msg);
            } else {
                face = Cell(
                    std::stoul(curFace[1]), std::stoul(curFace[2]),
                    std::stoul(curFace[3]));
                _faceList.push_back(face);
            }
        }
        std::getline(_plyFile, _line);
    }
}

void PLYReader2::_createMesh()
{
    ITKPoint p;
    uint32_t point_cnt = 0;
    for (auto& cur : _pointList) {
        p[0] = cur.x;
        p[1] = cur.y;
        p[2] = cur.z;
        _outMesh->SetPoint(point_cnt, p);
        if (_pointNorm) {
            ITKPixel q;
            q[0] = cur.nx;
            q[1] = cur.ny;
            q[2] = cur.nz;
            _outMesh->SetPointData(point_cnt, q);
        }
        point_cnt++;
    }
    uint32_t face_cnt = 0;
    for (auto& cur : _faceList) {
        ITKCell::CellAutoPointer cellpointer;
        cellpointer.TakeOwnership(new ITKTriangle);

        cellpointer->SetPointId(0, cur.v1);
        cellpointer->SetPointId(1, cur.v2);
        cellpointer->SetPointId(2, cur.v3);
        _outMesh->SetCell(face_cnt, cellpointer);
        face_cnt++;
    }
}
}  // namespace io
}  // namespace volcart
