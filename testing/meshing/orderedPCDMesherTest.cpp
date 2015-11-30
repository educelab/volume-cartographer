//
// Created by Ryan Taber on 11/6/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE orderedPCDMesher

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "shapes.h"
#include "orderedPCDMesher.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "parsingHelpers.h"


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
       // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //*cloud = pCloud;

        //assign outfile name
       // outfile = "fixOrderedPCD.pcd";

        //call orderedPCD()
        //volcart::meshing::orderedPCDMesher(cloud, outfile);

        std::cerr << "\nsetting up orderedPCDMesherTest objects" << std::endl;
    }

    ~orderedPCDFix(){ std::cerr << "\ncleaning up orderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> pCloud;
    volcart::shapes::Plane mesh;
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
    std::vector<VC_Cell> savedCells, currentCells;

    //parse the newly created and the saved .ply files and load the values into the
    //respective vectors. Note, this method is coming from parsingHelpers.cpp.
    volcart::testing::ParsingHelpers::parsePlyFile("testOrderedPCD.ply", currentPoints, currentCells);
    volcart::testing::ParsingHelpers::parsePlyFile("orderedPCDExample.ply", savedPoints, savedCells);
    

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

    BOOST_CHECK_EQUAL(savedCells.size(), currentCells.size());

    for (int f = 0; f < savedCells.size(); f++){

        //check the vertices within each face
        BOOST_CHECK_EQUAL(savedCells[f].v1, currentCells[f].v1);
        BOOST_CHECK_EQUAL(savedCells[f].v2, currentCells[f].v2);
        BOOST_CHECK_EQUAL(savedCells[f].v3, currentCells[f].v3);
    }
}




