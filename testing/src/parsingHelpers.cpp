//
// Created by Ryan Taber on 11/19/15.
//

#include "testing/parsingHelpers.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace volcart
{
namespace testing
{

void ParsingHelpers::parsePlyFile(
    const fs::path& filepath,
    std::vector<Vertex>& verts,
    std::vector<Cell>& faces)
{

    std::ifstream inputMesh(filepath.string());

    //   Test to see if .ply is open

    if (!inputMesh.is_open()) {
        exit(1);
    }

    //   Declare string to hold lines of ply file
    std::string line;
    getline(inputMesh, line);

    //   Vector will hold pieces of line delimited by space (filled by
    //   split_string())
    std::vector<std::string> plyLine;

    //   VC types to put store appropriate values read in from file
    Vertex plyVertex;
    Cell plyCell;

    int numVertices, numFaces, vertsPerFace, width, height;
    std::vector<std::string> typeOfPointInformation;
    bool headerFinished = false;
    bool isWidth = false;
    bool isHeight = false;

    // loop through file and get the appropriate data into the vectors
    while (!inputMesh.eof()) {

        plyLine = volcart::testing::ParsingHelpers::split_string(line);

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

            plyLine = volcart::testing::ParsingHelpers::split_string(line);
            while (plyLine[0] == "property") {

                typeOfPointInformation.push_back(plyLine[plyLine.size() - 1]);

                // get the next line
                line.clear();
                getline(inputMesh, line);
                plyLine = volcart::testing::ParsingHelpers::split_string(line);
            }

            // Get the face information
            if (plyLine[0] == "element" && plyLine[1] == "face") {
                numFaces = std::stoi(plyLine[2]);

                line.clear();
                getline(inputMesh, line);

                plyLine = volcart::testing::ParsingHelpers::split_string(line);

                //<< Do something with the property for the face information
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
                width = std::stoi(plyLine[0]);
                height = std::stoi(plyLine[1]);

                line.clear();
                getline(inputMesh, line);
                plyLine = volcart::testing::ParsingHelpers::split_string(line);
            }

            // Read in the vertex information
            for (int v = 0; v < numVertices; v++) {
                for (int i = 0; i < typeOfPointInformation.size(); i++) {

                    if (typeOfPointInformation[i] == "x")
                        plyVertex.x = std::stod(plyLine[i]);
                    else if (typeOfPointInformation[i] == "y")
                        plyVertex.y = std::stod(plyLine[i]);
                    else if (typeOfPointInformation[i] == "z")
                        plyVertex.z = std::stod(plyLine[i]);
                    else if (typeOfPointInformation[i] == "s")
                        plyVertex.s = std::stod(plyLine[i]);
                    else if (typeOfPointInformation[i] == "t")
                        plyVertex.t = std::stod(plyLine[i]);
                    else if (typeOfPointInformation[i] == "red")
                        plyVertex.r = std::stoi(plyLine[i]);
                    else if (typeOfPointInformation[i] == "green")
                        plyVertex.g = std::stoi(plyLine[i]);
                    else if (typeOfPointInformation[i] == "blue")
                        plyVertex.b = std::stoi(plyLine[i]);
                    else if (typeOfPointInformation[i] == "nx")
                        plyVertex.nx = std::stod(plyLine[i]);
                    else if (typeOfPointInformation[i] == "ny")
                        plyVertex.ny = std::stod(plyLine[i]);
                    else if (typeOfPointInformation[i] == "nz")
                        plyVertex.nz = std::stod(plyLine[i]);
                }

                // push vertex onto objPoints
                verts.push_back(plyVertex);

                line.clear();
                getline(inputMesh, line);
                plyLine = volcart::testing::ParsingHelpers::split_string(line);
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
                        if (v == 1)
                            plyCell.v1 = std::stoul(plyLine[v]);
                        else if (v == 2)
                            plyCell.v2 = std::stoul(plyLine[v]);
                        else if (v == 3)
                            plyCell.v3 = std::stoul(plyLine[v]);
                    }

                    faces.push_back(plyCell);

                    line.clear();
                    getline(inputMesh, line);
                    plyLine =
                        volcart::testing::ParsingHelpers::split_string(line);
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
void ParsingHelpers::parseObjFile(
    const fs::path& filepath,
    std::vector<Vertex>& points,
    std::vector<Cell>& cells)
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
    //   split_string())
    std::vector<std::string> objLine;

    //   VC types to put store appropriate values read in from file
    Vertex objVertex;
    Cell objCell;

    int numVertices, numFaces;
    int normalCounter = 0;
    std::vector<std::string> typeOfPointInformation;

    // loop through file and get the appropriate data into the vectors
    while (!inputMesh.eof()) {

        objLine = volcart::testing::ParsingHelpers::split_string(line);

        // skip comment lines
        if (objLine[0] == "#" && objLine[1] == "OBJ") {
            getline(inputMesh, line);
            continue;
        } else if (objLine[0] == "#" && objLine[1] == "Number") {

            // Number of points
            if (objLine[3] == "points")
                numVertices = std::stoi(objLine[4]);
            else if (objLine[3] == "cells")
                numFaces = std::stoi(objLine[4]);

            getline(inputMesh, line);
            continue;
        }

        //   Vertex
        else if (objLine[0] == "v" && objLine[1] != "n") {
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

/*
 *   Helper function to parse input lines based on space delimiter.
 *   Pieces are placed in string vector for later usage.
 *   Maybe this can be updated to take a char arg that can reference any
 * character
 *   delimiter necessary for a possible parse.
 */

std::vector<std::string> ParsingHelpers::split_string(std::string input)
{

    std::vector<std::string> output;
    auto nspaces = std::count(std::begin(input), std::end(input), ' ');
    output.reserve(nspaces);
    boost::split(
        output, input, boost::is_any_of("\t "), boost::token_compress_on);
    return output;
}

}  // testing
}  // volcart
