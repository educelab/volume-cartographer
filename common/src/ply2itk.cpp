//
// Created by Seth Parker on 4/27/15.
//

#include "common/io/ply2itk.h"
#include <boost/filesystem.hpp>

namespace volcart
{
namespace io
{

bool ply2itkmesh(const boost::filesystem::path& plyPath,
                 VC_MeshType::Pointer mesh,
                 int &width,
                 int &height)
{
    // open ply file
    std::ifstream plyFile(plyPath.string());
    if (!plyFile.is_open()) {
        std::cerr << "Open file " << plyPath << " failed." << std::endl;
        return false;
    }

    // parse ply file
    std::string line;

    int elementValue, aNumVertices, aNumFaces;
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
    double x, y, z, nx, ny, nz, s, t;
    int red, green, blue;

    // For Faces
    VC_CellType::CellAutoPointer cellpointer;
    int temp, p1, p2, p3;

    for (int i = 0; i < aElements.size(); ++i) {
        std::cout << "Reading element: " << aElementIDs[i]
                  << ", Number to be Read: " << aElements[i] << std::endl;
        for (int j = 0; j < aElements[i]; ++j) {
            // get the dimensions of the mesh
            if (aElementIDs[i] == "dimensions") {
                plyFile >> w >> h;
                width = w;
                height = h;
            }

            // read vertices
            if (aElementIDs[i] == "vertex") {
                VC_PointType p;
                VC_PixelType n;

                plyFile >> x >> y >> z >> nx >> ny >> nz >> s >> t >> red >>
                    green >> blue;
                p[0] = x;
                p[1] = y;
                p[2] = z;
                if (isnan(nx))
                    nx = 0;
                if (isnan(ny))
                    ny = 0;
                if (isnan(nz))
                    nz = 0;
                n[0] = nx;
                n[1] = ny;
                n[2] = nz;
                mesh->SetPoint(j, p);
                mesh->SetPointData(j, n);
            }

            // read faces
            if (aElementIDs[i] == "face") {
                plyFile >> temp >> p1 >> p2 >> p3;

                cellpointer.TakeOwnership(new VC_TriangleType);
                cellpointer->SetPointId(0, p1);
                cellpointer->SetPointId(1, p2);
                cellpointer->SetPointId(2, p3);
                mesh->SetCell(j, cellpointer);
            }
        }
    }

    // if we don't have the meshWidth and meshHeight, exit
    if (width == -1 || height == -1) {
        std::cerr
            << "ERROR: Mesh does not contain width and/or height metadata."
            << std::endl;
        return false;
    }

    plyFile.close();
    return true;
}

}  // namespace io
}  // namespace volcart
