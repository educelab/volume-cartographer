//
// Created by Hannah Hatch on 10/18/16.
//
#include "common/io/PlyReader2.h"

typedef std::pair<char,int> props;

namespace volcart
{
namespace io
{
bool PLYReader(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh)
{
    // open ply file
    std::ifstream plyFile(path.string());
    if (!plyFile.is_open()) {
        std::cerr << "Open file " << path << " failed." << std::endl;
        return false;
    }

    std::string line;
    int numofvertices;
    int numoffaces;

    getline(plyFile, line);

    while (line.find("element") == std::string::npos) {
        getline(plyFile, line);
    }

    if (line.find("vertex") == std::string::npos) {
        std::cerr << "No vertex information found" << std::endl;
        return false;
    } else
        numofvertices = line[2];

    int linecnt = 0;
    std::map<char, int> properties;
    std::map<std::string, int> point_norm;
    bool norm_check = false;
    getline(plyFile, line);

    while (line.find("element") == std::string::npos ||
           line.find("end_header") == std::string::npos) {
        if (line[2] == 'n') {
            norm_check = true;
            if (line[3] == 'x')
                point_norm["nx"] = linecnt;
            else if (line[3] == 'y')
                point_norm["ny"] = linecnt;
            else if (line[3] == 'z')
                point_norm["nz"] = linecnt;
        } else {
            properties[line[2]] = linecnt;
        }
        linecnt++;
        getline(plyFile, line);
    }

    if (line.find("faces") == std::string::npos) {
        std::cerr
            << "Warning: No face information found, reading in vertices only"
            << std::endl;
        numoffaces = 0;
    } else
        numoffaces = line[2];

    getline(plyFile, line);

    bool leadingchar = true;
    if (line.find("uchar") == std::string::npos)
        leadingchar = false;

    for (int i = 0; i < numofvertices; i++) {
        ITKPoint P;
        P[0] = std::stod(line.substr(properties['x'], ' '));
        P[1] = std::stod(line.substr(properties['y'], ' '));
        P[2] = std::stod(line.substr(properties['z'], ' '));
        mesh->SetPoint(i, P);
        if (norm_check) {
            ITKPixel Q;
            Q[0] = std::stod(line.substr(point_norm["nx"], ' '));
            Q[1] = std::stod(line.substr(point_norm["ny"], ' '));
            Q[2] = std::stod(line.substr(point_norm["nz"], ' '));
            mesh->SetPointData(i, Q);
        } else
            std::cerr << "No point normals found" << std::endl;
        getline(plyFile, line);
    }

    for (int i = 0; i < numoffaces; i++) {
        if (leadingchar) {
            int points_per_face = line[0];
            if (points_per_face != 3) {
                std::cerr << "Error: Not a triangular mesh" << std::endl;
                return false;
            } else {
                ITKCell::CellAutoPointer cellpointer;
                cellpointer.TakeOwnership(new ITKTriangle);
                for (int j = 0; j < points_per_face; ++j) {
                    cellpointer->SetPointId(j, line[j + 1]);
                }
                mesh->SetCell(i, cellpointer);
            }
        } else {
            std::cerr << "Point per face unknown, creating triangular mesh";
            ITKCell::CellAutoPointer cellpointer;
            cellpointer.TakeOwnership(new ITKTriangle);
            for (int j = 0; j < 3; ++j) {
                cellpointer->SetPointId(j, line[j + 1]);
            }
            mesh->SetCell(i, cellpointer);
        }
    }
    plyFile.close();
    return true;
}
}  // namespace io
}  // namespace volcart