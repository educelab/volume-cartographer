//
// Created by Hannah Hatch on 10/18/16.
//
/**@file PlyReader2.cpp */
#include "common/io/PlyReader2.h"

using Props = std::pair<char, int>;

namespace volcart
{
namespace io
{
bool PLYReader2(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh)
{
    // open ply file
    std::ifstream plyFile(path.string());
    if (!plyFile.is_open()) {
        std::cerr << "Open file " << path << " failed." << std::endl;
        return false;
    }

    std::string line;
    int numofvertices;
    std::string numofvertstr;
    int numoffaces;
    std::string numoffacestr;

    getline(plyFile, line);

    while (line.find("element", 0) == std::string::npos) {
        getline(plyFile, line);
    }

    if (line.find("vertex", 0) == std::string::npos) {
        std::cerr << "No vertex information found" << std::endl;
        return false;
    } else {
        numofvertstr = line.substr(line.find("vertex", 0) + 7);
        numofvertices = stoi(numofvertstr);
    }

    int linecnt = 0;
    std::map<char, int> properties;
    std::map<std::string, int> point_norm;
    bool norm_check = false;
    getline(plyFile, line);

    while (line.find("element", 0) > sizeof(line) ||
           line.find("end_header", 0) < sizeof(line)) {
        std::vector<std::string> curline;
        boost::split(
            curline, line, boost::is_any_of(" "), boost::token_compress_on);
        for (auto& element : curline) {
            if (element == "nx") {
                point_norm["nx"] = linecnt;
                norm_check = true;
            } else if (element == "ny") {
                point_norm["ny"] = linecnt;
            } else if (element == "nz") {
                point_norm["nz"] = linecnt;
            }
            if (element == "x") {
                properties['x'] = linecnt;
            } else if (element == "y") {
                properties['y'] = linecnt;
            } else if (element == "z") {
                properties['z'] = linecnt;
            }
        }

        // Added comment for testing

        linecnt++;
        getline(plyFile, line);
    }

    if (line.find("faces", 0) == std::string::npos &&
        line.find("face", 0) == std::string::npos) {
        std::cerr
            << "Warning: No face information found, reading in vertices only"
            << std::endl;
        numoffaces = 0;
    } else {
        std::vector<std::string> facenum;
        boost::split(
            facenum, line, boost::is_any_of(" "), boost::token_compress_on);
        numoffaces = stoi(facenum[2]);
    }

    getline(plyFile, line);

    bool leadingchar = true;
    if (line.find("uchar") == std::string::npos) {
        leadingchar = false;
    }
    getline(plyFile, line);
    if (line.find("end_header") != std::string::npos) {
        getline(plyFile, line);
    }
    for (int i = 0; i < numofvertices; i++) {
        std::vector<std::string> curpnt;
        boost::split(
            curpnt, line, boost::is_any_of(" "), boost::token_compress_on);
        ITKPoint P;
        P[0] = std::stod(curpnt[properties['x']]);
        P[1] = std::stod(curpnt[properties['y']]);
        P[2] = std::stod(curpnt[properties['z']]);
        mesh->SetPoint(i, P);
        if (norm_check) {
            ITKPixel Q;
            Q[0] = std::stod(curpnt[point_norm["nx"]]);
            Q[1] = std::stod(curpnt[point_norm["ny"]]);
            Q[2] = std::stod(curpnt[point_norm["nz"]]);
            mesh->SetPointData(i, Q);
        } else {
            std::cerr << "No point normals found" << std::endl;
        }
        getline(plyFile, line);
    }

    for (int i = 0; i < numoffaces; i++) {
        std::vector<std::string> curface;
        boost::split(
            curface, line, boost::is_any_of(" "), boost::token_compress_on);
        if (leadingchar) {
            int points_per_face = line[0] - '0';
            if (points_per_face != 3) {
                std::cerr << "Error: Not a triangular mesh" << std::endl;
                return false;
            } else {
                ITKCell::CellAutoPointer cellpointer;
                cellpointer.TakeOwnership(new ITKTriangle);
                for (int j = 0; j < points_per_face; ++j) {
                    unsigned long z = std::stoi(curface[j + 1]);
                    cellpointer->SetPointId(j, z);
                }
                mesh->SetCell(i, cellpointer);
            }
        } else {
            if (curface.size() == 3) {
                ITKCell::CellAutoPointer cellpointer;
                cellpointer.TakeOwnership(new ITKTriangle);
                for (int j = 0; j < 3; ++j) {
                    cellpointer->SetPointId(j, std::stoi(curface[j]));
                }
                mesh->SetCell(i, cellpointer);
            } else {
                std::cerr << "Error: Not a triangular mesh" << std::endl;
                return false;
            }
        }
        getline(plyFile, line);
    }
    plyFile.close();
    return true;
}
}  // namespace io
}  // namespace volcart