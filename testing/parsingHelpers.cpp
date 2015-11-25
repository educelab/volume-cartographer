//
// Created by Ryan Taber on 11/19/15.
//

#include <vc_defines.h>
#include "parsingHelpers.h"

namespace volcart{
namespace testing {

    void ParsingHelpers::parsePlyFile(std::string filepath, std::vector<VC_Vertex> &verts, std::vector<VC_Cell> &faces) {

        std::ifstream inputMesh;
        inputMesh.open(filepath);

        //   Test to see if .ply is open

        if (!inputMesh.is_open()) { exit(1); }

        //   Declare string to hold lines of ply file
        std::string line;
        getline(inputMesh, line);

        //   Vector will hold pieces of line delimited by space (filled by split_string())
        std::vector<std::string> plyLine;

        //   VC types to put store appropriate values read in from file
        VC_Vertex plyVertex;
        VC_Cell plyCell;

        int numVertices, numFaces, vertsPerFace;
        std::vector<std::string> typeOfPointInformation;
        bool headerFinished = false;

        //loop through file and get the appropriate data into the vectors
        while (!inputMesh.eof()) {

            plyLine = volcart::testing::ParsingHelpers::split_string(line);


            //skip header information not pertaining to vertex/face
            if (plyLine[0] == "ply"
                || plyLine[0] == "format"
                || plyLine[0] == "comment"
                || plyLine[0] == "obj_info") {

                getline(inputMesh, line);
                continue;
            }
                //Get number of vertices and faces
            else if (plyLine[0] == "element" && plyLine[1] == "vertex") {
                numVertices = stoi(plyLine[2]);

                line.clear();
                getline(inputMesh, line);

                plyLine = volcart::testing::ParsingHelpers::split_string(line);
                while (plyLine[0] == "property") {

                    typeOfPointInformation.push_back(plyLine[plyLine.size() - 1]);

                    //get the next line
                    line.clear();
                    getline(inputMesh, line);
                    plyLine = volcart::testing::ParsingHelpers::split_string(line);
                }

                //Get the face information
                if (plyLine[0] == "element" && plyLine[1] == "face") {
                    numFaces = stoi(plyLine[2]);

                    line.clear();
                    getline(inputMesh, line);

                    plyLine = volcart::testing::ParsingHelpers::split_string(line);
//                    if (plyLine[0] == "property") {
//
//                         std::string typeOfVertices = plyLine[3];
//                    }
                }
            }


            else if (plyLine[0] == "end_header") {
                headerFinished = true;
                getline(inputMesh, line);
                continue;
            }

            else if (headerFinished) {

                //Read in the vertex information
                for (int v = 0; v < numVertices; v++) {
                    for (int i = 0; i < typeOfPointInformation.size(); i++) {

                        if (typeOfPointInformation[i] == "x")
                            plyVertex.x = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "y")
                            plyVertex.y = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "z")
                            plyVertex.z = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "s")
                            plyVertex.s = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "t")
                            plyVertex.t = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "r")
                            plyVertex.r = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "g")
                            plyVertex.g = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "b")
                            plyVertex.b = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "nx")
                            plyVertex.nx = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "ny")
                            plyVertex.ny = stof(plyLine[i]);
                        else if (typeOfPointInformation[i] == "nz")
                            plyVertex.nz = stof(plyLine[i]);


                    }

                    //push vertex onto objPoints
                    verts.push_back(plyVertex);

                    line.clear();
                    getline(inputMesh, line);
                    plyLine = volcart::testing::ParsingHelpers::split_string(line);
                }

                vertsPerFace = 3;

                //Read in the face information
                for (int f = 0; f < numFaces; f++) {
                    for (int v = 1; v <= vertsPerFace; v++){

                        //only accounting for triangular faces currently
                        if (v == 1)
                            plyCell.v1 = stoul(plyLine[v]);
                        else if (v == 2)
                            plyCell.v2 = stoul(plyLine[v]);
                        else if (v == 3)
                            plyCell.v3 = stoul(plyLine[v]);
                    }

                    faces.push_back(plyCell);

                    line.clear();
                    getline(inputMesh, line);
                    plyLine = volcart::testing::ParsingHelpers::split_string(line);
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
void ParsingHelpers::parseObjFile(std::string filepath, std::vector<VC_Vertex> & points, std::vector<VC_Cell> &cells){

        std::ifstream inputMesh;
        inputMesh.open(filepath);

        //   Test to see if .ply is open
        if (!inputMesh.is_open()) { exit(1); }

        //   Declare string to hold lines of ply file
        std::string line;
        getline(inputMesh, line);

        //   Vector will hold pieces of line delimited by space (filled by split_string())
        std::vector<std::string> objLine;

        //   VC types to put store appropriate values read in from file
        VC_Vertex objVertex;
        VC_Cell objCell;

        int numVertices, numFaces;
        int normalCounter = 0;
        std::vector<std::string> typeOfPointInformation;

        //loop through file and get the appropriate data into the vectors
        while ( !inputMesh.eof() ) {

            objLine = volcart::testing::ParsingHelpers::split_string(line);

            //skip comment lines
            if (objLine[0] == "#" && objLine[1] == "OBJ"){
                getline(inputMesh, line);
                continue;
            }
            else if (objLine[0] == "#" && objLine[1] == "Number"){

                //Number of points
                if (objLine[3] == "points")
                    numVertices = stoi(objLine[4]);
                else if(objLine[3]  == "cells")
                    numFaces = stoi(objLine[4]);

                getline(inputMesh, line);
                continue;
            }

                //   Vertex
            else if (objLine[0] == "v" && objLine[1] != "n"){
                objVertex.x = std::stod(objLine[1]);
                objVertex.y = std::stod(objLine[2]);
                objVertex.z = std::stod(objLine[3]);

                //push the vertex onto the vector
                //these vertices will be updated in the 'vn' section
                points.push_back(objVertex);
            }

            // Add the normal information to the points vector
            else if (objLine[0] == "vn"){

                points[normalCounter].nx = std::stod(objLine[1]);
                points[normalCounter].ny = std::stod(objLine[2]);
                points[normalCounter].nz = std::stod(objLine[3]);

                //increment normal counter
                ++normalCounter;
            }

            // Face
            else if (objLine[0] == "f"){

                objCell.v1 = std::stoul(objLine[1]) - 1;
                objCell.v2 = std::stoul(objLine[2]) - 1;
                objCell.v3 = std::stoul(objLine[3]) - 1;

                //push the cell onto objCells vector
                cells.push_back(objCell);
            }

            // get next line of obj file
            line.clear();
            getline( inputMesh, line );
        }

        inputMesh.close();
}


/*
 *   Helper function to parse input lines based on space delimiter.
 *   Pieces are placed in string vector for later usage.
 *   Maybe this can be updated to take a char arg that can reference any character
 *   delimiter necessary for a possible parse.
 */

std::vector<std::string> ParsingHelpers::split_string(std::string input) {

    std::vector<std::string> line;

    size_t field_start = 0;
    size_t next_space;

    do {
        next_space = input.find(' ', field_start);

        if (next_space == std::string::npos) {
            line.push_back(input.substr(field_start));
        }

        else if (next_space - field_start != 0) {
            line.push_back(input.substr(field_start, next_space - field_start));
        }

        field_start = next_space + 1;

    } while (next_space != std::string::npos);

    return line;

}



} // testing
} // volcart




