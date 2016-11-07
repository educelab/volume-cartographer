//
// Created by Hannah Hatch on 10/18/16.
//
/**@file PlyReader2.cpp */
#include <common/types/Exceptions.h>
#include "common/io/PlyReader2.h"

using Props = std::pair<char, int>;

namespace volcart
{
namespace io
{
PLYReader2::PLYReader2() {};

PLYReader2::PLYReader2(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh) {
    _inputPath = path;
    _outMesh = mesh;
}

bool PLYReader2::read() {
    _plyFile.open(_inputPath.string());
    if(!_plyFile.is_open())
    {
        auto msg = "Open file " + _inputPath.string() + " failed.";
        throw volcart::IOException(msg);
        return false;
    }
    _parseHeader();
    if(_facesFirst){
        _readFaces();
        _readPoints();
    }
    else {
        _readPoints();
        _readFaces();
    }
    _plyFile.close();
    _createMesh();
    return true;
}

void PLYReader2::_parseHeader() {
    std::vector<std::string> facenum;
    std::vector<std::string> vertnum;
    std::getline(_plyFile, _line);
    while(_line.find("element",0) == std::string::npos){
        std::getline(_plyFile,_line);
    }
    //Assumes that if there's face information, there's also vertex information
    if(_line.find("vertex",0) == std::string::npos && _line.find("face", 0) != std::string::npos){
        _facesFirst = true;
        boost::split(
                facenum,_line,boost::is_any_of(" "), boost::token_compress_on
        );
        _numOfFaces=std::stoi(facenum[2]);
        std::getline(_plyFile, _line);
        if(_line.find("uchar") == std::string::npos){
            _leadingChar = false;
        }
        getline(_plyFile,_line);
        boost::split(vertnum,_line,boost::is_any_of(" "), boost::token_compress_on);
        _numOfVertices=std::stoi(vertnum[2]);
        std::getline(_plyFile, _line);
        int currentLine = 0;
        while(_line.find("element",0) > sizeof(_line) || _line.find("end_header",0) < sizeof(_line)){
            std::vector<std::string> curline;
            boost::split(curline,_line,boost::is_any_of(" "), boost::token_compress_on);
            for(auto& element : curline){
                if(element == "nx") {
                    properties["nx"] = currentLine;
                } else if (element == "ny"){
                    properties["ny"] = currentLine;
                } else if (element == "nz"){
                    properties["nz"] = currentLine;
                } else if (element == "x"){
                    properties["x"] = currentLine;
                } else if(element == "y"){
                    properties["y"] = currentLine;
                } else if (element == "z"){
                    properties["z"] = currentLine;
                } else if (element == "r"){
                    properties["r"] = currentLine;
                } else if(element == "g"){
                    properties["g"] = currentLine;
                } else if (element == "b"){
                    properties["b"] = currentLine;
                }//if-else
            }//for
            getline(_plyFile, _line);
            currentLine++;
        }//while
    }
    else if(_line.find("vertex",0) == std::string::npos && _line.find("face", 0) == std::string::npos){
        auto msg = "No header information, file cannot be parsed";
        throw volcart::IOException(msg);
    }
    else{
        _facesFirst = false;
        boost::split(vertnum,_line,boost::is_any_of(" "), boost::token_compress_on);
        _numOfVertices=std::stoi(vertnum[2]);
        std::getline(_plyFile, _line);
        int currentLine = 0;
        while(_line.find("element",0) > sizeof(_line) || _line.find("end_header",0) < sizeof(_line)){
            std::vector<std::string> curline;
            boost::split(curline,_line,boost::is_any_of(" "), boost::token_compress_on);
            for(auto& element : curline){
                if(element == "nx") {
                    properties["nx"] = currentLine;
                } else if (element == "ny"){
                    properties["ny"] = currentLine;
                } else if (element == "nz"){
                    properties["nz"] = currentLine;
                } else if (element == "x"){
                    properties["x"] = currentLine;
                } else if(element == "y"){
                    properties["y"] = currentLine;
                } else if (element == "z"){
                    properties["z"] = currentLine;
                } else if (element == "r"){
                    properties["r"] = currentLine;
                }else if(element == "g"){
                    properties["g"] = currentLine;
                }else if (element == "b"){
                    properties["b"] = currentLine;
                }//if-else
            }//for
            getline(_plyFile, _line);
            currentLine++;
        }//while
        boost::split(
                facenum,_line,boost::is_any_of(" "), boost::token_compress_on
        );
        if(_line.find("face",0) == std::string::npos){
            std::cerr << "Warning: No face information found, reading in vertices only" << std::endl;
            _numOfFaces = 0;
        }
        else{
            _numOfFaces=std::stoi(facenum[2]);
            std::getline(_plyFile, _line);
            if(_line.find("uchar") == std::string::npos) {
                _leadingChar = false;
            }
        }
        getline(_plyFile,_line);
    }//else
    while(_line.find("end_header",0) != std::string::npos){
        getline(_plyFile, _line);
    }
    getline(_plyFile, _line);
}//ParseHeader

void PLYReader2::_readPoints() {
    for(int i = 0; i < _numOfVertices; i ++){
        Point curPoint;
        std::vector<std::string> curLine;
        boost::split(curLine,_line,boost::is_any_of(" "), boost::token_compress_on);
        curPoint.x = std::stod(curLine[properties["x"]]);
        curPoint.y = std::stod(curLine[properties["y"]]);
        curPoint.z = std::stod(curLine[properties["z"]]);
        if(properties.find("nx") != properties.end()){
            curPoint.nx = std::stod(curLine[properties["nx"]]);
            curPoint.ny = std::stod(curLine[properties["ny"]]);
            curPoint.nz = std::stod(curLine[properties["nz"]]);
        }
        if(properties.find("r") != properties.end()){
            curPoint.r = curLine[properties["r"]];
            curPoint.g = curLine[properties["g"]];
            curPoint.b = curLine[properties["b"]];
        }
        _pointList.push_back(curPoint);
        getline(_plyFile, _line);
    }
    getline(_plyFile, _line);
}

void PLYReader2::_readFaces() {
    for (int i = 0; i < _numOfFaces; i++) {
        std::vector<std::string> curFace;
        std::tuple<int, int, int> face;
        boost::split(curFace, _line, boost::is_any_of(" "), boost::token_compress_on);
        if (_leadingChar) {
            int points_per_face = _line[0] - '0';
            if (points_per_face != 3) {
                auto msg = "Error: Not a Triangular Mesh";
                throw volcart::IOException(msg);
            } else {
                face = std::make_tuple(std::stoi(curFace[1]), std::stoi(curFace[2]), std::stoi(curFace[3]));
                _faceList.push_back(face);
            }
        } else {
            if (curFace.size() == 3) {
                face = std::make_tuple(std::stoi(curFace[1]), std::stoi(curFace[2]), std::stoi(curFace[3]));
                _faceList.push_back(face);
            } else {
                auto msg = "Error: Not a Triangular Mesh";
                throw volcart::IOException(msg);
            }
        }
        getline(_plyFile, _line);
    }
}

void PLYReader2::_createMesh() {
    ITKPoint P;
    int point_cnt = 0;
    for(auto& cur : _pointList){
        P[0] = cur.x;
        P[1] = cur.y;
        P[2] = cur.z;
        _outMesh -> SetPoint(point_cnt,P);
        if(cur.nx != 0){
            ITKPixel Q;
            Q[0] = cur.nx;
            Q[1] = cur.ny;
            Q[2] = cur.nz;
            _outMesh -> SetPointData(point_cnt,Q);
        }
        point_cnt++;
    }
    int face_cnt = 0;
    for (auto& cur: _faceList) {
        ITKCell::CellAutoPointer cellpointer;
        cellpointer.TakeOwnership(new ITKTriangle);
        for(int j =0; j < 3; j ++){
            if (j == 0)
            {
                cellpointer->SetPointId(j,std::get<0>(cur));
            }
            else if (j == 1){
                cellpointer->SetPointId(j,std::get<1>(cur));
            } else if (j == 2){
                cellpointer->SetPointId(j,std::get<2>(cur));
            }
        }
        _outMesh -> SetCell(face_cnt, cellpointer);
    }
}
}  // namespace io
}  // namespace volcart