//
// Created by Ryan Taber on 9/25/15.
//

#define BOOST_TEST_MODULE sample

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Plane.hpp"

/************************************************************************************
 *                                                                                  *
 *  This is a sample test using boost.test unit testing framework. The ultimate
 * *
 *  goal of this file is the following: *
 *                                                                                  *
 *        1. check whether a testing mesh, created by *
 *           core/shapes/Plane.h, can be written into *
 *           an object file by core/io/OBJWriter.cpp. *
 *                                                                                  *
 *        2. read contents of obj file and compare data with testing mesh *
 *                                                                                  *
 *  This file is broken up into a testing fixture, meshFix, which initializes
 * the   *
 *  objects used in each of the two test cases. *
 *                                                                                  *
 *  writeTest (test case): *
 *                                                                                  *
 *      attempts to write a testing mesh to file and compares final output path
 * *
 *      as a check for success. Note, this test always outputs the file as *
 *      "output.obj" because there is no texture information included when
 * writing. *
 *                                                                                  *
 *  compareElements (test case): *
 *                                                                                  *
 *      Attempts to read in information from obj file using boost::path
 * variable.   *
 *      As data is read in from the obj file, collections of points and faces
 * are   *
 *      created to compare against points and faces created during
 * initialization   *
 *      of the testing mesh by meshFix. The test then compares all non-commented
 * *
 *      data from the file against the testing mesh data to ensure equality. *
 *                                                                                  *
 * Input: *
 *     No required inputs for this sample test. *
 *                                                                                  *
 * Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general  *
 *     number of testing errors is output. *
 *                                                                                  *
 * ************************************************************************************/

//  helper function to split lines read in from obj file  //
std::vector<std::string> split_string(std::string);

/*
 * This builds objects for each of the test cases below that reference
 * the fixture as their second argument
 *
 */

struct meshFix {

    meshFix()
    {

        // create the mesh for all the test cases to use
        _mesh = mesh.itkMesh();

        std::cerr << "setting up mesh" << std::endl;
    }

    ~meshFix() { std::cerr << "cleaning up meshFix" << std::endl; }

    // file path and objwriter to be used in cases
    volcart::io::OBJWriter mesh_writer;
    boost::filesystem::path objPath;
    ITKMesh::Pointer _mesh;
    volcart::shapes::Plane mesh;
};

// Test for checking successful write
BOOST_FIXTURE_TEST_CASE(writeTest, meshFix)
{

    std::cout << "Writing object" << std::endl;

    mesh_writer.setPath("nothing");
    // mesh_writer.setUVMap( uvMap );
    // mesh_writer.setTexture( uvImg );

    objPath = mesh_writer.getPath();
    objPath = boost::filesystem::absolute(objPath);

    // mesh_writer.write() runs validate() as well, but this lets us handle a
    // mesh that can't be validated.
    if (mesh_writer.validate())
        mesh_writer.write();
    else {
        mesh_writer.setPath("output.obj");
        mesh_writer.setMesh(_mesh);
        mesh_writer.write();
    }

    // check the file path from the mesh_writer.write() call above
    // compare() returns 0 only if paths are same value
    // lexicographically-speaking
    // checking "output.obj" here because the mesh_writer shouldn't validate in
    // the current case
    BOOST_CHECK_EQUAL(mesh_writer.getPath().compare("output.obj"), 0);
}

BOOST_FIXTURE_TEST_CASE(compareElements, meshFix)
{

    std::vector<Vertex> testPoints = mesh.getPoints();
    std::vector<Cell> testCells = mesh.getCells();

    //   Get absolute path of obj file   //

    objPath = mesh_writer.getPath();
    objPath = boost::filesystem::absolute(objPath);

    //   Now to get the points and cells from the saved obj file   //

    std::ifstream inputMesh;
    inputMesh.open(
        objPath.string() +
        "/output.obj");  // probably better way of assigning filename

    //   Test to see if output.obj is open     //

    if (!inputMesh.is_open()) {
        BOOST_CHECK(false);
    }

    //   Declare string to hold lines of obj file   //
    std::string line;
    getline(inputMesh, line);

    //   Vector will hold pieces of line delimited by space (filled by
    //   split_string()
    std::vector<std::string> objLine;

    //   Declare vectors to hold mesh points and cells from obj file
    std::vector<Vertex> objPoints;
    std::vector<Cell> objCells;

    //   VC types to put store appropriate values read in from file
    Vertex objVertex;
    Cell objCell;

    // loop through file and get the appropriate data into the vectors
    while (!inputMesh.eof()) {

        objLine = split_string(line);

        // skip comment lines
        if (objLine[0] == "#") {
            getline(inputMesh, line);
            continue;
        }

        //   Vertex
        else if (objLine[0] == "v" && objLine[1] != "n") {
            objVertex.x = std::stod(objLine[1]);
            objVertex.y = std::stod(objLine[2]);
            objVertex.z = std::stod(objLine[3]);
        }

        // Normal
        else if (objLine[0] == "vn") {
            objVertex.nx = std::stod(objLine[1]);
            objVertex.ny = std::stod(objLine[2]);
            objVertex.nz = std::stod(objLine[3]);

            // push vertex onto objPoints
            objPoints.push_back(objVertex);
        }

        // Face
        else if (objLine[0] == "f") {

            // obj file format of 'v/vt/vn' 'v/vt/vn' 'v/vt/vn'
            // seems we should be able to just get the 'v' from each of these
            // sets and subtract one
            // before assigning to v1,v2,v3 for each cell.

            for (size_t faceVertex = 1; faceVertex < 4; faceVertex++) {

                size_t lpos = 0;
                size_t pos;

                pos = objLine[faceVertex].find("//", lpos);

                // assign the v1,v2,v3 values //

                if (faceVertex == 1)
                    objCell.v1 = std::stoul(
                        objLine[faceVertex].substr(lpos, pos - lpos));
                else if (faceVertex == 2)
                    objCell.v2 = std::stoul(
                        objLine[faceVertex].substr(lpos, pos - lpos));
                else
                    objCell.v3 = std::stoul(
                        objLine[faceVertex].substr(lpos, pos - lpos));
            }
            // push the cell onto objCells vector
            objCells.push_back(objCell);
        }

        // get next line of obj file
        line.clear();
        getline(inputMesh, line);
    }

    // Now to test the objPoints created during fixture init vs points read in
    // from file.
    for (size_t p = 0; p < testPoints.size(); p++) {

        BOOST_TEST_MESSAGE("Checking Point " << p);

        // checking the x,y,z,nx,ny,nz components

        BOOST_CHECK_EQUAL(testPoints[p].x, objPoints[p].x);
        BOOST_CHECK_EQUAL(testPoints[p].y, objPoints[p].y);
        BOOST_CHECK_EQUAL(testPoints[p].z, objPoints[p].z);

        BOOST_CHECK_EQUAL(testPoints[p].nx, objPoints[p].nx);
        BOOST_CHECK_EQUAL(testPoints[p].ny, objPoints[p].ny);
        BOOST_CHECK_EQUAL(testPoints[p].nz, objPoints[p].nz);
    }

    // Now to test the objPoints created during fixture init vs points read in
    // from file.
    for (size_t c = 0; c < testCells.size(); c++) {

        BOOST_TEST_MESSAGE("Checking Cell: " << c);

        // checking the v1,v2,v3 components

        BOOST_CHECK_EQUAL(testCells[c].v1, objCells[c].v1 - 1);
        BOOST_CHECK_EQUAL(testCells[c].v2, objCells[c].v2 - 1);
        BOOST_CHECK_EQUAL(testCells[c].v3, objCells[c].v3 - 1);
    }
}

/*
 *   Helper function to parse input lines based on space delimiter.
 *   Pieces are placed in string vector for later usage.
 *   Maybe this can be updated to take a char arg that can reference any
 * character
 *   delimiter necessary for a possible parse.
 */

std::vector<std::string> split_string(std::string input)
{
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
