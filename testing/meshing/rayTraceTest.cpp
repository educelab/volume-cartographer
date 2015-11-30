//
// Created by Ryan Taber on 11/30/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE rayTrace


#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "shapes.h"
#include "rayTrace.h"
#include "vc_defines.h"
#include "parsingHelpers.h"

/************************************************************************************
 *                                                                                  *
 *  orderedPCDMesherTest.cpp - tests the functionality of                           *
 *  v-c/meshing/orderedPCDMesher.cpp with the ultimate goal of the following:       *
 *                                                                                  *
 *     Given the same input point cloud, does a saved PLY file match a current      *
 *     execution of orederedPCDMesher().                                            *
 *                                                                                  *
 *  This file is broken up into a test fixture orderedPCDFix which initialize       *
 *  the objects used in any subsequent fixture test cases.                          *
 *                                                                                  *
 *  Test Cases:                                                                     *
 *  1. orderedPCDTest (fixture test case)                                           *
 *                                                                                  *                                                                            *
 *  Input:                                                                          *
 *     No required inputs for the test cases. Any test objects are created          *
 *     internally by orderedPCDFix() or within the test cases themselves.           *
 *                                                                                  *
 *  Test-Specific Output:                                                           *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output to console.                               *
 *                                                                                  *
 *  Miscellaneous:                                                                  *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/

/*
 * Purpose of rayTraceFix:
 *      - generate a point cloud consisting of PointXYZRGB points
 *      - call orderedPCDMesher() on this point cloud and write to file
 */

struct rayTraceFix {

    rayTraceFix() {

        //generate the curved mesh
        iMesh = _mesh.itkMesh();

        //call rayTrace() and assign results
        traceResults = volcart::meshing::rayTrace(iMesh, traceDir, width, height, uvMap);

        //write the rayTrace() results to file

        std::cerr << "\nsetting up rayTraceTest objects" << std::endl;
    }

    ~rayTraceFix(){ std::cerr << "\ncleaning up rayTraceTest objects" << std::endl; }



    std::vector<cv::Vec6f> traceResults;

    // Essential data structure to return points and normals
    std::vector<cv::Vec6f> intersections;
    VC_MeshType::Pointer iMesh;
    volcart::shapes::Plane _mesh;
    std::map<int, cv::Vec2d> uvMap;

    int traceDir = 0; //default direction is anything != 1
    int width, height;

};

/*
 * Test to see that a saved PLY file from fixture matches a recalled orderedPCDMesher()
 * using the same input point cloud.
 */
BOOST_FIXTURE_TEST_CASE(savedRayTraceComparison, rayTraceFix){

    //First, write the fixture-created rayTrace data to file
    int numPoints = traceResults.size();

    std::ofstream meshFile;
    meshFile.open("testRayTrace.ply");

    std::cout << "Writing rayTrace results to file..." << std::endl;

    // write header
    meshFile << "ply" << std::endl
    << "format ascii 1.0" << std::endl
    << "comment Created by particle simulation https://github.com/viscenter/registration-toolkit" << std::endl
    << "element vertex " << numPoints << std::endl
    << "property float x" << std::endl
    << "property float y" << std::endl
    << "property float z" << std::endl
    << "property float nx" << std::endl
    << "property float ny" << std::endl
    << "property float nz" << std::endl
    << "element face 0" << std::endl
    << "property list uchar int vertex_indices" << std::endl
    << "end_header" << std::endl;

    // write vertex information
    for (int i = 0; i < numPoints; i++) {

        // x y z nx ny nz
        meshFile << traceResults[i](0) << " "
                 << traceResults[i](1)  << " "
                 << traceResults[i](2)  << " "
                 << traceResults[i](3)  << " "
                 << traceResults[i](4)  << " ";

                 // Hack to get rid of "-0" values that appeared in the
                 // saved file the first time this was run

                 if (traceResults[i](5) == -0) {
                     meshFile << "0 " << std::endl;
                 }else{
                     meshFile << traceResults[i](5) << " " << std::endl;
                 }
    }

    meshFile.close();

    //Read in the saved data created by rayTraceExample.cpp
    std::vector<VC_Vertex> savedPoints, currentPoints;
    std::vector<VC_Cell> savedCells, currentCells;      //cell vecs unused but init for the parse call

    //Read in the .ply files
    //parsePlyFile() found in parsingHelpers.cpp
    volcart::testing::ParsingHelpers::parsePlyFile("savedRayTraceData.ply", savedPoints, savedCells);
    volcart::testing::ParsingHelpers::parsePlyFile("testRayTrace.ply", currentPoints, currentCells);


    //Compare the saved and test-case-created resampling for equivalency
    BOOST_CHECK_EQUAL(savedPoints.size(), currentPoints.size());

    //loop over points
    for (int p = 0; p < savedPoints.size(); p++){

        BOOST_CHECK_EQUAL(savedPoints[p].x, currentPoints[p].x);
        BOOST_CHECK_EQUAL(savedPoints[p].y, currentPoints[p].y);
        BOOST_CHECK_EQUAL(savedPoints[p].z, currentPoints[p].z);
        BOOST_CHECK_EQUAL(savedPoints[p].nx, currentPoints[p].nx);
        BOOST_CHECK_EQUAL(savedPoints[p].ny, currentPoints[p].ny);
        BOOST_CHECK_EQUAL(savedPoints[p].nz, currentPoints[p].nz);

    }

}
