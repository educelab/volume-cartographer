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

/*
 *
 * THIS IS GARBAGE.
 *
 * CAN NOT FIND A TEST CASE THAT ACTUALLY CHANGES THE POINTS IN THE RESAMPLED
 * POINT CLOUD FROM THE ORIGINAL....SO IT'S CURRENTLY DISABLED IN THE CMAKELIST.
 *
 *
 */


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
 *      This test is simply looking into the points of the resulting PC.            *
 *                                                                                  *
 *  Properties (test case):                                                         *
 *                                                                                  *
 *      Takes a point cloud created from fixture via testingMesh::pointCloud()      *
 *      Checks the resulting PC's properties, i.e., height, width, size, etc.,      *
 *      against the input PC.                                                       *
 *                                                                                  *
 *  Magnitude (test case):                                                          *
 *                                                                                  *
 *      Takes a point cloud created from fixture via testingMesh::pointCloud()      *
 *      Checks that the normals in the resulting PC are unit normals.               *
 *                                                                                  *
 *  Input:                                                                          *
 *     No required inputs for the test cases. Any test objects are created          *
 *     internally by resampleFix().                                                 *
 *                                                                                  *
 *  Test-Specific Output:                                                           *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output.                                          *
 *                                                                                  *
 *  Miscellaneous:                                                                  *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/


/*
 * Boost test fixture - resampleFix
 *       - builds a pcl::PointCloud<pcl::PointNormal> object for
 *         PCTest test case
 */

struct resampleFix {

    resampleFix() {

        pCloud = mesh.pointCloudXYZ();

        std::cerr << "\nsetting up resamplePCTest objects" << std::endl;
    }

    ~resampleFix(){ std::cerr << "\ncleaning up resamplePCTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZ> pCloud;
    volcart::testing::testingMesh mesh;

    // Note, radius between (0,1] returns an empty PC
    // Additionally, radius less than or equal to 0 will throw error
    // during resampling as defined by PCL
    double radius;

    //init new PC
    pcl::PointCloud<pcl::PointNormal> newCloud;

};

BOOST_FIXTURE_TEST_CASE(PCTest, resampleFix){

    //init new PC
    //pcl::PointCloud<pcl::PointNormal> newCloud;

    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pCloud;


/*************************************************
 * Looking at the original and resulting PC data *
 *************************************************/

    //place cloud points in vector
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > cloudData = cloud->points;
    std::cerr << "Original Cloud Points: " << std::endl;

    for (int c = 0; c < cloudData.size(); c++) {
        std::cerr << "Point " << c << ": "
        << cloudData[c].x << " | "
        << cloudData[c].y << " | "
        << cloudData[c].z << " | " << std::endl;
    }

    //Let's loop through various radius values to see how the resulting PC looks
    for (radius = 50.0; radius <  55.0; radius ++) {

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

/******************************************************
 * Check core properties of original and resampled PC *
 ******************************************************/

BOOST_FIXTURE_TEST_CASE(properties, resampleFix){

    std::cerr << "Testing resampled properties..." << std::endl;
    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pCloud;

    //set search radius and call resample()
    radius = 2.0;
    newCloud = volcart::meshing::resamplePointCloud(cloud, radius);

    //check that pc props match
    BOOST_CHECK_EQUAL(newCloud.height, cloud->height);
    BOOST_CHECK_EQUAL(newCloud.width, cloud->width);
    BOOST_CHECK_EQUAL(newCloud.is_dense, cloud->is_dense);
    BOOST_CHECK_EQUAL(newCloud.size(), cloud->size());

}

/*******************************************************************
 * Check magnitude of normals in resampled PC based on original PC *
 *******************************************************************/
BOOST_FIXTURE_TEST_CASE(magnitude, resampleFix){

    std::cerr << "Testing resampled normal magnitudes..." << std::endl;
    std::cerr << "Normal magnitude tested against 1.0 with tolerance of 0.0000001" << std::endl;

    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pCloud;

    //set search radius and call resample()
    radius = 2.0;
    newCloud = volcart::meshing::resamplePointCloud(cloud, radius);

    //normals should be unit normals, so we're testing to confirm this below
    //for each of the normals created by resampling.

    float magnitude;
    int pctr = 0;

    //loop through the newCloud and check normal exists for each point
    for (auto n = newCloud.points.begin(); n != newCloud.points.end(); n++){

        magnitude = sqrt( pow((n->normal_x),2) + pow((n->normal_y),2)  + pow((n->normal_z),2) );

       // std::cerr << "Normal " << pctr <<  *n << ": " << magnitude << std::endl;

        //checks magnitude with tolerance of 0.0000001 against 1.0
        BOOST_CHECK_CLOSE(magnitude, 1.0, 0.0000001);

        pctr++;

    }

}