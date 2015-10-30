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
 *        1. Comparing a resampled Point Cloud (PC) to an input PC                  *
 *           - the output PC should contain only the neighbors within the           *
 *             passed radius parameter in resamplePointCloud().                     *
 *           - RESULTING NORMALS NEED TO BE CHECKED HOW?                            *
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

        pCloud = mesh.pointCloud();

        std::cerr << "setting up resamplePCTest objects" << std::endl;
    }

    ~resampleFix(){ std::cerr << "cleaning up resamplePCTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointNormal> pCloud;
    volcart::testing::testingMesh mesh;

};

BOOST_FIXTURE_TEST_CASE(PCTest, resampleFix){

    //init new PC
    pcl::PointCloud<pcl::PointNormal> newCloud;

    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(&pCloud);

    //Let's look at the original points
    //place cloud points in vector
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > cloudData = cloud->points;
    std::cerr << "Original Cloud Points: " << std::endl;

    for (int c = 0; c < cloudData.size(); c++){
        std::cerr << "Point " << c << ": "
        << cloudData[c].x << " | "
        << cloudData[c].y << " | "
        << cloudData[c].z << " | "
        << cloudData[c].normal_x << " | "
        << cloudData[c].normal_y << " | "
        << cloudData[c].normal_z << " | "  << std::endl;
    }
    double radius;

    //Let's loop through various radius values to see how the resulting PC looks
    for (radius = 5; radius >= 0; radius--) {

        //call resample PC
        newCloud = volcart::meshing::resamplePointCloud(cloud, radius);

        //place newCloud points in vector
        std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > newCloudData = newCloud.points;
        std::cerr << "newCloud Points for Radius: " << radius << std::endl;

        //Look at the points
        for (int i = 0; i < newCloudData.size(); i ++){
            std::cerr << "Point " << i << ": "
                      << newCloudData[i].x << " | "
                      << newCloudData[i].y << " | "
                      << newCloudData[i].z << " | "
                      << newCloudData[i].normal_x << " | "
                      << newCloudData[i].normal_y << " | "
                      << newCloudData[i].normal_z << " | "  << std::endl;
        }

    }

    //holder for test
    BOOST_CHECK(true);


}