//
// Created by Seth Parker on 4/27/15.
//

#include "common/io/PLYReader.h"
#include <boost/filesystem.hpp>

using namespace volcart::io;
namespace fs = boost::filesystem;

bool PLYReader(fs::path path, volcart::ITKMesh::Pointer mesh)
{
    // open ply file
    std::ifstream plyFile(path.string());
    if (!plyFile.is_open()) {
        std::cerr << "Open file " << path << " failed." << std::endl;
        return false;
    }

    // parse ply file
    std::string line;

    int elementValue;
    std::string elementID;
    std::vector<int> aElements;
    std::vector<std::string> aElementIDs, parsed;

    getline(plyFile, line);
    // Read until we hit the end of the header
    while (line.find("end_header") == std::string::npos) {
        // For each "element" line in the ply header, parse that line to get the
        // name of that
        // element and the number of that element that should be in the file
        if (line.find("element") != std::string::npos) {
            size_t lpos = 0;
            size_t pos = line.find(" ", lpos);

            while (pos != std::string::npos) {
                parsed.push_back(line.substr(lpos, pos - lpos));
                lpos = pos + 1;
                pos = line.find(" ", pos + 1);
            }

            // pickup the last element
            parsed.push_back(line.substr(lpos, pos - lpos));

            // assumes element declaration in ply header == "element [elementID]
            // [num_of_element]"
            elementID = parsed[1];
            elementValue = std::atoi(parsed[2].c_str());

            aElementIDs.push_back(elementID);
            aElements.push_back(elementValue);

            parsed.clear();
        }
        getline(plyFile, line);
    }

    // For dims
    int w, h;

    // For Vertices
    double x, y, z, nx, ny, nz;

    // For Faces
    volcart::ITKCell::CellAutoPointer cellpointer;
    int temp, p1, p2, p3;

    for (size_t i = 0; i < aElements.size(); ++i) {
        std::cout << "Reading element: " << aElementIDs[i]
                  << ", Number to be Read: " << aElements[i] << std::endl;
        for (int j = 0; j < aElements[i]; ++j) {
            // get the dimensions of the mesh
            if (aElementIDs[i] == "dimensions") {
                plyFile >> w >> h;
            }

            // read vertices
            if (aElementIDs[i] == "vertex") {
                volcart::ITKPoint p;
                volcart::ITKPixel n;

                plyFile >> x >> y >> z >> nx >> ny >> nz;
                p[0] = x;
                p[1] = y;
                p[2] = z;
                if (std::isnan(nx)) {
                    nx = 0;
                }
                if (std::isnan(ny)) {
                    ny = 0;
                }
                if (std::isnan(nz)) {
                    nz = 0;
                }
                n[0] = nx;
                n[1] = ny;
                n[2] = nz;
                mesh->SetPoint(j, p);
                mesh->SetPointData(j, n);
            }

            // read faces
            if (aElementIDs[i] == "face") {
                plyFile >> temp >> p1 >> p2 >> p3;

                cellpointer.TakeOwnership(new volcart::ITKTriangle);
                cellpointer->SetPointId(0, p1);
                cellpointer->SetPointId(1, p2);
                cellpointer->SetPointId(2, p3);
                mesh->SetCell(j, cellpointer);
            }
        }
    }

    plyFile.close();
    return true;
}
