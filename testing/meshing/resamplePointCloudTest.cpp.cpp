//
// Created by Ryan Taber on 10/28/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE resamplePointCloud

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "resamplePointCloud.h"


/************************************************************************************
 *                                                                                  *
 *  resamplePointCloudTest.cpp - tests the functionality of                         *
 *  v-c/meshing/resamplePointCloud.cpp with the ultimate goal of the following:     *
 *                                                                                  *
 *        1.
 *                                                                                  *
 *                                                                                  *
 *  This file is broken up into two test fixtures resampleFix which initialize      *
 *  the objects used in the test cases.                                             *
 *                                                                                  *
 *  PCTest (test case):                                                             *
 *                                                                                  *
 *      Takes a point cloud created from fixture via testingMesh::pointCloud()      *
 *
 *                                                                                  *
 *                                                                                  *
 * Input:                                                                           *
 *     No required inputs for this sample test. Any test objects are created        *
 *     internally.                                                                  *
 *                                                                                  *
 * Test-Specific Output:                                                            *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output.                                          *
 *                                                                                  *
 * Miscellaneous:                                                                   *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/


/*
 * Boost test fixture - resampleFix
 *       - builds a pcl::PointCloud<pcl::PointNormal> object for
 *         PCTest test case
 */

struct resampleFix {

    resampleFix() {

        cloud = mesh.pointCloud();

        std::cerr << "setting up resamplePCTest objects" << std::endl;
    }

    ~resampleFix(){ std::cerr << "cleaning up resamplePCTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointNormal> cloud;
    volcart::testing::testingMesh mesh;

};

BOOST_FIXTURE_TEST_CASE(PCTest, resampleFix){


    //holder for test
    BOOST_CHECK(true);


}