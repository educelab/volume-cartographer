#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <vector>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/String.hpp"
#include "vc/testing/ParsingHelpers.hpp"

namespace fs = volcart::filesystem;

namespace volcart::testing
{

void ParsingHelpers::ParsePLYFile(
    const fs::path& filepath,
    std::vector<SimpleMesh::Vertex>& verts,
    std::vector<SimpleMesh::Cell>& faces)
{

    std::ifstream inputMesh(filepath.string());

    //   Test to see if .ply is open
    if (!inputMesh.is_open()) {
        std::exit(1);
    }

    //   Declare string to hold lines of ply file
    std::string line;
    getline(inputMesh, line);

    //   Vector will hold pieces of line delimited by space (filled by
    //   SplitString())
    std::vector<std::string> plyLine;

    //   VC types to put store appropriate values read in from file
    SimpleMesh::Vertex plyVertex;
    SimpleMesh::Cell plyCell;

    int numVertices{}, numFaces{}, vertsPerFace{};
    std::vector<std::string> typeOfPointInformation;
    bool headerFinished = false;
    bool isWidth = false;
    bool isHeight = false;

    // loop through file and get the appropriate data into the vectors
    while (!inputMesh.eof()) {

        plyLine = split(line, '\t', ' ');

        // skip header information not pertaining to vertex/face
        if (plyLine[0] == "ply" || plyLine[0] == "format" ||
            plyLine[0] == "comment" || plyLine[0] == "obj_info") {

            getline(inputMesh, line);
            continue;
        }

        // Check to see if dimensions are included in the file
        else if (plyLine[0] == "element" && plyLine[1] == "dimensions") {

            if (plyLine[2] == "1") {
                isWidth = true;
                isHeight = true;
            }

            // for 3d mesh...add something here...might need a depth variable

            getline(inputMesh, line);
            continue;

        }
        // Get number of vertices and faces
        else if (plyLine[0] == "element" && plyLine[1] == "vertex") {
            numVertices = std::stoi(plyLine[2]);

            line.clear();
            getline(inputMesh, line);

            plyLine = split(line, '\t', ' ');
            while (plyLine[0] == "property") {

                typeOfPointInformation.push_back(plyLine.back());

                // get the next line
                line.clear();
                getline(inputMesh, line);
                plyLine = split(line, '\t', ' ');
            }

            // Get the face information
            if (plyLine[0] == "element" && plyLine[1] == "face") {
                numFaces = std::stoi(plyLine[2]);

                line.clear();
                getline(inputMesh, line);

                plyLine = split(line, '\t', ' ');

                ///<< Do something with the property for the face information
                // here >>

                // if (plyLine[0] == "property") {
                //
                //    std::string typeOfVertices = plyLine[3];
                //}
            }
        }

        else if (plyLine[0] == "end_header") {
            headerFinished = true;
            getline(inputMesh, line);
            continue;
        }

        else if (headerFinished) {

            // Read in the dimension info if it exists
            if (isWidth && isHeight) {
                line.clear();
                getline(inputMesh, line);
                plyLine = split(line, '\t', ' ');
            }

            // Read in the vertex information
            for (int v = 0; v < numVertices; v++) {
                for (std::size_t i = 0; i < typeOfPointInformation.size();
                     i++) {

                    if (typeOfPointInformation[i] == "x") {
                        plyVertex.x = std::stod(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "y") {
                        plyVertex.y = std::stod(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "z") {
                        plyVertex.z = std::stod(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "s") {
                        plyVertex.s = std::stod(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "t") {
                        plyVertex.t = std::stod(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "red") {
                        plyVertex.r = std::stoi(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "green") {
                        plyVertex.g = std::stoi(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "blue") {
                        plyVertex.b = std::stoi(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "nx") {
                        plyVertex.nx = std::stod(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "ny") {
                        plyVertex.ny = std::stod(plyLine[i]);
                    } else if (typeOfPointInformation[i] == "nz") {
                        plyVertex.nz = std::stod(plyLine[i]);
                    }
                }

                // push vertex onto objPoints
                verts.push_back(plyVertex);

                line.clear();
                getline(inputMesh, line);
                plyLine = split(line, '\t', ' ');
            }

            /*
             * Note, the ply file format written by the particle simulation
             * through
             * the viscenter/registration -toolkit doesn't seem to write face
             * information out,
             * so it's necessary to check for numFaces as greater than 0 before
             * parsing any face information
             */

            if (numFaces > 0) {
                vertsPerFace = std::stoi(plyLine[0]);

                // Read in the face information
                for (int f = 0; f < numFaces; f++) {
                    for (int v = 1; v <= vertsPerFace; v++) {

                        // only accounting for triangular faces currently
                        if (v == 1) {
                            plyCell.v1 = std::stoul(plyLine[v]);
                        } else if (v == 2) {
                            plyCell.v2 = std::stoul(plyLine[v]);
                        } else if (v == 3) {
                            plyCell.v3 = std::stoul(plyLine[v]);
                        }
                    }

                    faces.push_back(plyCell);

                    line.clear();
                    getline(inputMesh, line);
                    plyLine = split(line, '\t', ' ');
                }
            }
        }

        // get next line of file
        line.clear();
        getline(inputMesh, line);
    }

    inputMesh.close();
}

/*
 *   Helper function to parse an obj file.
 */
void ParsingHelpers::ParseOBJFile(
    const fs::path& filepath,
    std::vector<SimpleMesh::Vertex>& points,
    std::vector<SimpleMesh::Cell>& cells)
{

    std::ifstream inputMesh(filepath.string());

    //   Test to see if file is open
    if (!inputMesh.is_open()) {
        exit(1);
    }

    //   Declare string to hold lines of ply file
    std::string line;
    getline(inputMesh, line);

    //   Vector will hold pieces of line delimited by space (filled by
    //   SplitString())
    std::vector<std::string> objLine;

    //   VC types to put store appropriate values read in from file
    SimpleMesh::Vertex objVertex;
    SimpleMesh::Cell objCell;

    int normalCounter = 0;
    std::vector<std::string> typeOfPointInformation;

    // loop through file and get the appropriate data into the vectors
    while (!inputMesh.eof()) {

        objLine = split(line, '\t', ' ');
        if (objLine.empty()) {
            continue;
        }

        // skip comment lines
        if (objLine[0][0] == '#') {
            std::getline(inputMesh, line);
            continue;
        }

        //   Vertex
        else if (objLine[0] == "v") {
            objVertex.x = std::stod(objLine[1]);
            objVertex.y = std::stod(objLine[2]);
            objVertex.z = std::stod(objLine[3]);

            // push the vertex onto the vector
            // these vertices will be updated in the 'vn' section
            points.push_back(objVertex);
        }

        // Add the normal information to the points vector
        else if (objLine[0] == "vn") {
            points[normalCounter].nx = std::stod(objLine[1]);
            points[normalCounter].ny = std::stod(objLine[2]);
            points[normalCounter].nz = std::stod(objLine[3]);

            // increment normal counter
            ++normalCounter;
        }

        // Face
        else if (objLine[0] == "f") {
            objCell.v1 = std::stoul(objLine[1]) - 1;
            objCell.v2 = std::stoul(objLine[2]) - 1;
            objCell.v3 = std::stoul(objLine[3]) - 1;

            // push the cell onto objCells vector
            cells.push_back(objCell);
        }

        // get next line of obj file
        line.clear();
        getline(inputMesh, line);
    }

    inputMesh.close();
}
}  // namespace volcart::testing
