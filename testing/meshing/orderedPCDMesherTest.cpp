//
// Created by Ryan Taber on 11/6/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE orderedPCDMesher

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "orderedPCDMesher.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


/************************************************************************************
 *                                                                                  *
 *  orderedPCDMesherTest.cpp - tests the functionality of                           *
 *  v-c/meshing/orderedPCDMesher.cpp with the ultimate goal of the following:       *
 *                                                                                  *
 *     Given the same input point cloud two PolygonMesh meshes created by           *
 *     poissonReconstruction() should represent the same reconstructed surface.     *
 *                                                                                  *
 *                                                                                  *
 *  This file is broken up into a test fixture poissonFix which initialize          *
 *  the objects used in the fixture test cases.                                     *
 *                                                                                  *
 *  Test Cases:                                                                     *
 *  1. poissonTest (fixture test case)                                              *
 *  2. surfaceComparison (fixture test case)                                        *
 *  3. emptyCloud (auto test case)                                                  *
 *  4. onePoint (auto test case)                                                    *
 *  5. moreThanOnePoint (auto test case)                                            *
 *                                                                                  *                                                                            *
 *  Input:                                                                          *
 *     No required inputs for the test cases. Any test objects are created          *
 *     internally by poissonFix() or within the test cases themselves.              *
 *                                                                                  *
 *  Test-Specific Output:                                                           *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output to console.                               *
 *                                                                                  *
 *  Miscellaneous:                                                                  *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/

//prototype for helper functions below
std::vector<VC_Vertex> parsePlyFile(std::string filename);
std::vector<std::string> split_string(std::string input);

/*
 * Purpose of orderedPCDFix:
 *      - generate a point cloud consisting of PointXYZRGB points
 *      - call orderedPCDMesher() on this point cloud and write to file
 */

struct orderedPCDFix {

    orderedPCDFix() {

        //Create point cloud from mesh
        pCloud = mesh.pointCloudXYZRGB();

        //convert pCloud to Ptr for orderedPCD() call
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloud = pCloud;

        //assign outfile name
        outfile = "fixOrderedPCD.pcd";

        //call orderedPCD()
        volcart::meshing::orderedPCDMesher(cloud, outfile);

        std::cerr << "\nsetting up orderedPCDMesherTest objects" << std::endl;
    }

    ~orderedPCDFix(){ std::cerr << "\ncleaning up orderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> pCloud;
    volcart::testing::testingMesh mesh;
    std::string outfile;

};

/*
 * Test to see that a saved PCD file from fixture matches a recalled orderedPCDMesher()
 * using the same input point cloud.
 */
BOOST_FIXTURE_TEST_CASE(orderedPCDTest, orderedPCDFix){


    //convert pCloud to Ptr for orderedPCD() call
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *testCloud = pCloud;

    //just using literal for outfile here
    volcart::meshing::orderedPCDMesher(testCloud, "testOrderedPCD.ply");

    std::vector<VC_Vertex> savedPoints, currentPoints;

    currentPoints = parsePlyFile("testOrderedPCD.ply");
    savedPoints = parsePlyFile("orderedPCDExample.ply");

    //Check sizes of the two data sets
    BOOST_CHECK_EQUAL(savedPoints.size(), currentPoints.size());

    //Loop through the points within the clouds to check equivalency
    for (int p = 0; p < savedPoints.size(); p++){

        //check x,y,z,nx,ny,nz,s,t,r,g,b values
        BOOST_CHECK_EQUAL(savedPoints[p].x, currentPoints[p].x);
        BOOST_CHECK_EQUAL(savedPoints[p].y, currentPoints[p].y);
        BOOST_CHECK_EQUAL(savedPoints[p].z, currentPoints[p].z);
        BOOST_CHECK_EQUAL(savedPoints[p].nx, currentPoints[p].nx);
        BOOST_CHECK_EQUAL(savedPoints[p].ny, currentPoints[p].ny);
        BOOST_CHECK_EQUAL(savedPoints[p].nz, currentPoints[p].nz);
        BOOST_CHECK_EQUAL(savedPoints[p].s, currentPoints[p].s);
        BOOST_CHECK_EQUAL(savedPoints[p].t, currentPoints[p].t);
        BOOST_CHECK_EQUAL(savedPoints[p].r, currentPoints[p].r);
        BOOST_CHECK_EQUAL(savedPoints[p].g, currentPoints[p].g);
        BOOST_CHECK_EQUAL(savedPoints[p].b, currentPoints[p].b);

    }
}

/* Helper function used to parse the custom ply writer we have
 * in orderedPCDMesher.cpp
 *
 * Input: string filename
 * Output: vector<VC_Vertex> plyPoints
 *
 * Usage: called in test case above to check the
 *        equivalency of points written by orderedPCDExample.cpp
 *        against a test-case-created file, "testOrderedPCD.ply"
 */

std::vector<VC_Vertex> parsePlyFile(std::string filename){

    std::ifstream inputMesh;
    inputMesh.open(filename);

    //   Test to see if orderedPCDExample.ply is open

    if (!inputMesh.is_open()) { BOOST_CHECK(false);}

    //   Declare string to hold lines of ply file
    std::string line;
    getline(inputMesh,line);

    //   Vector will hold pieces of line delimited by space (filled by split_string())
    std::vector<std::string> plyLine;

    //   Declare vectors to hold mesh points and cells from obj file
    std::vector<VC_Vertex> plyPoints;

    //   VC types to put store appropriate values read in from file
    VC_Vertex plyVertex ;

    float width, height;
    //loop through file and get the appropriate data into the vectors
    while ( !inputMesh.eof() ) {

        plyLine = split_string(line);

        //skip header information
        if (plyLine[0] == "ply"
        || plyLine[0] == "format"
        || plyLine[0] == "comment"
        || plyLine[0] == "element"
        || plyLine[0] == "property"
        || plyLine[0] == "end_header" ){

            getline(inputMesh, line);
            continue;
        }


        // Width height line
        // Not doing anything with this currently

        else if (plyLine.size() == 2){
            width = std::stof(plyLine[0]);
            height = std::stof(plyLine[1]);
        }


        // Otherwise, we have a vertex
        else if (plyLine.size() == 11){

            plyVertex.x = std::stof(plyLine[0]);
            plyVertex.y = std::stof(plyLine[1]);
            plyVertex.z = std::stof(plyLine[2]);
            plyVertex.nx = std::stof(plyLine[3]);
            plyVertex.ny = std::stof(plyLine[4]);
            plyVertex.nz = std::stof(plyLine[5]);
            plyVertex.s = std::stof(plyLine[6]);
            plyVertex.t = std::stof(plyLine[7]);
            plyVertex.r = std::stod(plyLine[8]);
            plyVertex.g = std::stod(plyLine[9]);
            plyVertex.b = std::stod(plyLine[10]);

            //push vertex onto objPoints
            plyPoints.push_back(plyVertex);
        }

        //TODO: Add face parsing???

        // get next line of obj file
        line.clear();
        getline( inputMesh, line );
    }

    return plyPoints;
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




