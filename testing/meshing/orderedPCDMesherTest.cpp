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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloud = pCloud;

        //assign outfile name
        outfile = "fixOrderedPCD.pcd";

        //call orderedPCD()
        volcart::meshing::orderedPCDMesher(cloud, outfile)

        std::cerr << "\nsetting up orderedPCDMesherTest objects" << std::endl;
    }

    ~orderedPCDFix(){ std::cerr << "\ncleaning up orderedPCDMesherTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointNormal> pCloud;
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
    volcart::meshing::orderedPCDMesher(testCloud, "testOrderedPCD.pcd");

}



