/**@file PLYReader2.cpp */

#include "common/io/PLYReader2.h"

using Props = std::pair<char, int>;

namespace volcart
{
namespace io
{

bool PLYReader2::read()
{
    // Resets values of member variables in case of 2nd reading
    _pointList.clear();
    _faceList.clear();
    _properties.clear();
    _elementsList.clear();
    _outMesh = ITKMesh::New();
    _numOfVertices = 0;
    _numOfFaces = 0;
    _leadingChar = true;
    _pointNorm = false;

    int skippedElementCnt = 0;

    _plyFile.open(_inputPath.string());
    if (!_plyFile.is_open()) {
        auto msg = "Open file " + _inputPath.string() + " failed.";
        throw volcart::IOException(msg);
    }
    _parseHeader();
    for (auto& cur : _elementsList) {
        if (cur == "vertex") {
            _readPoints();
        } else if (cur == "face") {
            _readFaces();
        } else {
            int curSkip = _skippedLine[skippedElementCnt];
            for (int i = 0; i < curSkip; i++) {
                getline(_plyFile, _line);
            }
            skippedElementCnt++;
        }
    }
    _plyFile.close();
    _createMesh();
    return true;
}

void PLYReader2::_parseHeader()
{
    int currentLine;
    std::getline(_plyFile, _line);
    while (_line != "end_header") {
        if (_line.find("element") != std::string::npos) {
            std::vector<std::string> splitLine;
            boost::split(
                splitLine, _line, boost::is_any_of(" "),
                boost::token_compress_on);
            _elementsList.push_back(splitLine[1]);
            if (splitLine[1] == "vertex") {
                _numOfVertices = std::stoi(splitLine[2]);
            } else if (splitLine[1] == "face") {
                _numOfFaces = std::stoi(splitLine[2]);
            } else {
                _skippedLine.push_back(std::stoi(splitLine[2]));
            }
            getline(_plyFile, _line);
            boost::split(
                splitLine, _line, boost::is_any_of(" "),
                boost::token_compress_on);
            currentLine = 0;
            while (splitLine[0] == "property") {
                if (splitLine[1] == "list") {
                    if (_line.find("uchar") == std::string::npos) {
                        _leadingChar = false;
                    }
                }
                // Not sure how to handle if it's not the vertices or faces
                else {
                    if (splitLine[2] == "nx") {
                        _pointNorm = true;
                    }
                    _properties[splitLine[2]] = currentLine;
                }
                getline(_plyFile, _line);
                currentLine++;
                boost::split(
                    splitLine, _line, boost::is_any_of(" "),
                    boost::token_compress_on);
            }
        } else {
            getline(_plyFile, _line);
        }
    }
    if (_numOfFaces == 0) {
        std::cerr << "Warning: No face information found" << std::endl;
    }
    std::getline(_plyFile, _line);

}  // ParseHeader

void PLYReader2::_readPoints()
{
    for (int i = 0; i < _numOfVertices; i++) {
        volcart::Vertex curPoint;
        std::vector<std::string> curLine;
        boost::split(
            curLine, _line, boost::is_any_of(" "), boost::token_compress_on);
        curPoint.x = std::stod(curLine[_properties["x"]]);
        curPoint.y = std::stod(curLine[_properties["y"]]);
        curPoint.z = std::stod(curLine[_properties["z"]]);
        if (_properties.find("nx") != _properties.end()) {
            curPoint.nx = std::stod(curLine[_properties["nx"]]);
            curPoint.ny = std::stod(curLine[_properties["ny"]]);
            curPoint.nz = std::stod(curLine[_properties["nz"]]);
        }
        if (_properties.find("r") != _properties.end()) {
            curPoint.r = stoi(curLine[_properties["r"]]);
            curPoint.g = stoi(curLine[_properties["g"]]);
            curPoint.b = stoi(curLine[_properties["b"]]);
        }
        _pointList.push_back(curPoint);
        std::getline(_plyFile, _line);
    }
}

void PLYReader2::_readFaces()
{
    for (int i = 0; i < _numOfFaces; i++) {
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
