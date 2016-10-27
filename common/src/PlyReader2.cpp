//
// Created by Hannah Hatch on 10/18/16.
//
#include "common/io/PlyReader2.h"

typedef std::pair<char,int> props;

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

    while (line.find("element",0) == std::string::npos) {
        getline(plyFile, line);
    }

    if (line.find("vertex",0) == std::string::npos) {
        std::cerr << "No vertex information found" << std::endl;
        return false;
    } else
    {
        numofvertstr = line.substr(line.find("vertex",0) +7);
        numofvertices = stoi(numofvertstr);
    }

    int linecnt = 0;
    std::map<char, int> properties;
    std::map<std::string, int> point_norm;
    bool norm_check = false;
    getline(plyFile, line);

    while (line.find("element",0) > sizeof(line) ||
           line.find("end_header",0) < sizeof(line)) {
        if (line.find("nx",0) != std::string::npos) {
            point_norm["nx"] = linecnt*2;
            norm_check = true;
        }
        else if (line.find("ny",0) != std::string::npos)
            point_norm["ny"] = linecnt *2;
        else if (line.find("nz",0) != std::string::npos)
            point_norm["nz"] = linecnt *2;
        if(line.find(" x",0) != std::string::npos) {
            properties['x'] = linecnt *2 ;
        }
        else if (line.find(" y",0) != std::string::npos)
            properties['y'] = linecnt *2;
        else if (line.find(" z",0) != std::string::npos)
            properties['z'] = linecnt *2;
        linecnt++;
        getline(plyFile, line);
    }

    if (line.find("faces",0) == std::string::npos && line.find("face",0) == std::string::npos) {
        std::cerr
            << "Warning: No face information found, reading in vertices only"
            << std::endl;
        numoffaces = 0;
    } else
    {
        if(line.find("face",0) == std::string::npos)
        {
            numoffacestr = line.substr(line.find("faces",0) +6);
            numoffaces = stoi(numoffacestr);
        } else{
            numoffacestr = line.substr(line.find("face",0) + 5);
            numoffaces = stoi(numoffacestr);
        }

    }

    getline(plyFile, line);

    bool leadingchar = true;
    if (line.find("uchar") == std::string::npos)
        leadingchar = false;
    getline(plyFile,line);
    if(line.find("end_header") != std::string::npos)
        getline(plyFile,line);
    for (int i = 0; i < numofvertices; i++) {
        ITKPoint P;
        P[0] = std::stod(line.substr(properties['x'], line.find(' ', properties['x'])));
        P[1] = std::stod(line.substr(properties['y'], line.find(' ', properties['y'])));
        P[2] = std::stod(line.substr(properties['z'], line.find(' ', properties['z'])));
        mesh->SetPoint(i, P);
        if (norm_check) {
            ITKPixel Q;
            Q[0] = std::stod(line.substr(point_norm["nx"],line.find(' ', point_norm["nx"])));
            Q[1] = std::stod(line.substr(point_norm["ny"], line.find(' ', point_norm["ny"])));
            Q[2] = std::stod(line.substr(point_norm["nz"], line.find(' ', point_norm["nz"])));
            mesh->SetPointData(i, Q);
        } else
            std::cerr << "No point normals found" << std::endl;
        getline(plyFile, line);
    }

    for (int i = 0; i < numoffaces; i++) {
        int chartcnt = 0;
        if (leadingchar) {
            int points_per_face = line[0] - '0';
            if (points_per_face != 3) {
                std::cerr << "Error: Not a triangular mesh" << std::endl;
                return false;
            } else {
                chartcnt++;
                ITKCell::CellAutoPointer cellpointer;
                cellpointer.TakeOwnership(new ITKTriangle);
                for (int j = 0; j < points_per_face; ++j) {
                    unsigned long z = std::stoi(line.substr(j+chartcnt+1, line.find(' ', j+chartcnt+1)));
                    cellpointer->SetPointId(j,z);
                    chartcnt++;
                }
                mesh->SetCell(i, cellpointer);
            }
        } else {
            std::cerr << "Point per face unknown, creating triangular mesh";
            ITKCell::CellAutoPointer cellpointer;
            cellpointer.TakeOwnership(new ITKTriangle);
            for (int j = 0; j < 3; ++j) {
                cellpointer->SetPointId(j, (int(line[j +1] - '0')+ 2*chartcnt));
                chartcnt++;
            }
            mesh->SetCell(i, cellpointer);
        }
        getline(plyFile, line);
    }
    plyFile.close();
    return true;
}
}  // namespace io
}  // namespace volcart