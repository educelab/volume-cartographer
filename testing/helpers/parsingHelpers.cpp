//
// Created by Ryan Taber on 11/19/15.
//

#include "parsingHelpers.h"

void parsePlyFile(std::string filename, std::vector<VC_Vertex> &verts, std::vector<VC_Cell> &faces){

    std::ifstream inputMesh;
    inputMesh.open(filename);

    //   Test to see if .ply is open

    if (!inputMesh.is_open()) { exit(1);}

    //   Declare string to hold lines of ply file
    std::string line;
    getline(inputMesh,line);

    //   Vector will hold pieces of line delimited by space (filled by split_string())
    std::vector<std::string> plyLine;

    //   Declare vectors to hold mesh points and cells from obj file
    std::vector<VC_Vertex> plyPoints;
    std::vector<VC_Cell> plyCells;

    //   VC types to put store appropriate values read in from file
    VC_Vertex plyVertex ;
    VC_Cell plyCell;

    int numVertices, numFaces;
    std::vector<std::string> typeOfPointInformation;
    bool headerFinished = false;

    //loop through file and get the appropriate data into the vectors
    while ( !inputMesh.eof() ) {

        plyLine = split_string(line);

        //skip header information not pertaining to vertex/face
        if (plyLine[0] == "ply"
            || plyLine[0] == "format"
            || plyLine[0] == "comment") {

            getline(inputMesh, line);
            continue;
        }
            //Get number of vertices
        else if (plyLine[0] == "element" && plyLine[1] == "vertex") {
            numVertices = stoi(plyLine[2]);

            line.clear();
            getline(inputMesh, line);
            while (plyLine[0] == "property") {

                typeOfPointInformation.push_back(plyLine[plyLine.size() - 1]);

                //get the next line
                line.clear();
                getline(inputMesh, line);
                plyLine = split_string(line);
            }

        }
            //Get the face information
        else if (plyLine[0] == "element" && plyLine[1] == "face") {
            numFaces = stoi(plyLine[2]);
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

                    plyVertex.typeOfInformation[i] = std::stof(plyLine[i]);
                }

                //push vertex onto objPoints
                plyPoints.push_back(plyVertex);

                line.clear();
                getline(inputMesh, line);
                plyLine = split_string(line);
            }

            int numPointsInCell = plyLine.size();

            //Read in the face information
            for (int f = 0; f < numFaces; f++) {

                //assuming triangular cells
                //can add a line to check the number of vertices per cell
                //by checking the first number on the line
                plyCell.v1 = std::stoul(plyLine[1]);
                plyCell.v2 = std::stoul(plyLine[2]);
                plyCell.v3 = std::stoul(plyLine[3]);

                plyCells.push_back(plyCell);

                line.clear();
                getline(inputMesh, line);
                plyLine = split_string(line);
            }

        }

        // get next line of file
        line.clear();
        getline(inputMesh, line);

    }

}


/*
 *   Helper function to parse input lines based on space delimiter.
 *   Pieces are placed in string vector for later usage.
 *   Maybe this can be updated to take a char arg that can reference any character
 *   delimiter necessary for a possible parse.
 */

std::vector<std::string> split_string(std::string input)
{
    std::vector<std::string> line;

    size_t field_start = 0;
    size_t next_space;

    do{
        next_space = input.find(' ', field_start);

        if (next_space == std::string::npos){
            line.push_back(input.substr(field_start));
        }

        else if (next_space - field_start != 0){
            line.push_back(input.substr(field_start, next_space - field_start));
        }

        field_start = next_space + 1;

    } while (next_space != std::string::npos);

    return line;

}





