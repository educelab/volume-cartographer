/**@file PLYReader.cpp */

#include "core/io/PLYReader.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem/operations.hpp>

#include "core/types/Exceptions.hpp"

using Props = std::pair<char, int>;

using namespace volcart;
using namespace volcart::io;
namespace fs = boost::filesystem;

bool PLYReader::read()
{
    if (inputPath_.empty() || !fs::exists(inputPath_)) {
        auto msg = "File not provided or does not exist.";
        throw volcart::IOException(msg);
    }

    // Resets values of member variables in case of 2nd reading
    pointList_.clear();
    faceList_.clear();
    properties_.clear();
    elementsList_.clear();
    outMesh_ = ITKMesh::New();
    numVertices_ = 0;
    numFaces_ = 0;
    hasLeadingChar_ = true;
    hasPointNorm_ = false;

    int skippedElementCnt = 0;

    plyFile_.open(inputPath_.string());
    if (!plyFile_.is_open()) {
        auto msg = "Open file " + inputPath_.string() + " failed.";
        throw volcart::IOException(msg);
    }
    parse_header_();
    for (auto& cur : elementsList_) {
        if (cur == "vertex") {
            read_points_();
        } else if (cur == "face") {
            read_faces_();
        } else {
            int curSkip = skippedLine_[skippedElementCnt];
            for (int i = 0; i < curSkip; i++) {
                std::getline(plyFile_, line_);
            }
            skippedElementCnt++;
        }
    }
    plyFile_.close();
    create_mesh_();
    return true;
}

void PLYReader::parse_header_()
{
    int currentLine;
    std::getline(plyFile_, line_);
    while (line_ != "end_header") {
        if (line_.find("element") != std::string::npos) {
            std::vector<std::string> splitLine;
            boost::split(
                splitLine, line_, boost::is_any_of(" "),
                boost::token_compress_on);
            elementsList_.push_back(splitLine[1]);
            if (splitLine[1] == "vertex") {
                numVertices_ = std::stoi(splitLine[2]);
            } else if (splitLine[1] == "face") {
                numFaces_ = std::stoi(splitLine[2]);
            } else {
                skippedLine_.push_back(std::stoi(splitLine[2]));
            }
            std::getline(plyFile_, line_);
            boost::split(
                splitLine, line_, boost::is_any_of(" "),
                boost::token_compress_on);
            currentLine = 0;
            while (splitLine[0] == "property") {
                if (splitLine[1] == "list") {
                    hasLeadingChar_ = line_.find("uchar") != std::string::npos;
                }
                // Not sure how to handle if it's not the vertices or faces
                else {
                    if (splitLine[2] == "nx") {
                        hasPointNorm_ = true;
                    }
                    properties_[splitLine[2]] = currentLine;
                }
                std::getline(plyFile_, line_);
                currentLine++;
                boost::split(
                    splitLine, line_, boost::is_any_of(" "),
                    boost::token_compress_on);
            }
        } else {
            std::getline(plyFile_, line_);
        }
    }
    if (numFaces_ == 0) {
        std::cerr << "Warning: No face information found" << std::endl;
    }
    std::getline(plyFile_, line_);

}  // ParseHeader

void PLYReader::read_points_()
{
    for (int i = 0; i < numVertices_; i++) {
        volcart::Vertex curPoint;
        std::vector<std::string> curLine;
        boost::split(
            curLine, line_, boost::is_any_of(" "), boost::token_compress_on);
        curPoint.x = std::stod(curLine[properties_["x"]]);
        curPoint.y = std::stod(curLine[properties_["y"]]);
        curPoint.z = std::stod(curLine[properties_["z"]]);
        if (properties_.find("nx") != properties_.end()) {
            curPoint.nx = std::stod(curLine[properties_["nx"]]);
            curPoint.ny = std::stod(curLine[properties_["ny"]]);
            curPoint.nz = std::stod(curLine[properties_["nz"]]);
        }
        if (properties_.find("r") != properties_.end()) {
            curPoint.r = stoi(curLine[properties_["r"]]);
            curPoint.g = stoi(curLine[properties_["g"]]);
            curPoint.b = stoi(curLine[properties_["b"]]);
        }
        pointList_.push_back(curPoint);
        std::getline(plyFile_, line_);
    }
}

void PLYReader::read_faces_()
{
    for (int i = 0; i < numFaces_; i++) {
        std::vector<std::string> curFace;
        volcart::Cell face;
        boost::split(
            curFace, line_, boost::is_any_of(" "), boost::token_compress_on);
        if (hasLeadingChar_) {
            int pointsPerFace = std::stoi(curFace[0]);
            if (pointsPerFace != 3) {
                auto msg = "Error: Not a Triangular Mesh";
                throw volcart::IOException(msg);
            } else {
                face = Cell(
                    std::stoul(curFace[1]), std::stoul(curFace[2]),
                    std::stoul(curFace[3]));
                faceList_.push_back(face);
            }
        } else {
            if (curFace.size() != 3) {
                auto msg = "Error: Not a Triangular Mesh";
                throw volcart::IOException(msg);
            } else {
                face = Cell(
                    std::stoul(curFace[1]), std::stoul(curFace[2]),
                    std::stoul(curFace[3]));
                faceList_.push_back(face);
            }
        }
        std::getline(plyFile_, line_);
    }
}

void PLYReader::create_mesh_()
{
    ITKPoint p;
    uint32_t pointCount = 0;
    for (auto& cur : pointList_) {
        p[0] = cur.x;
        p[1] = cur.y;
        p[2] = cur.z;
        outMesh_->SetPoint(pointCount, p);
        if (hasPointNorm_) {
            ITKPixel q;
            q[0] = cur.nx;
            q[1] = cur.ny;
            q[2] = cur.nz;
            outMesh_->SetPointData(pointCount, q);
        }
        pointCount++;
    }
    uint32_t faceCount = 0;
    for (auto& cur : faceList_) {
        ITKCell::CellAutoPointer cellpointer;
        cellpointer.TakeOwnership(new ITKTriangle);

        cellpointer->SetPointId(0, cur.v1);
        cellpointer->SetPointId(1, cur.v2);
        cellpointer->SetPointId(2, cur.v3);
        outMesh_->SetCell(faceCount, cellpointer);
        faceCount++;
    }
}
