//
// Created by Ryan Taber on 10/28/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE resamplePointCloud

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "shapes.h"
#include "resamplePointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

/************************************************************************************
 *                                                                                  *
 *  resamplePointCloudTest.cpp - tests the functionality of                         *
 *  v-c/meshing/resamplePointCloud.cpp                                              *
 *                                                                                  *
 *                                                                                  *
 *  This file is broken up into a test fixture, resampleFix, which initializes      *
 *  some of the objects used in the fixture test cases:                             *
 *                                                                                  *
 *  1. PCTest:                                                                      *
 *                                                                                  *
 *      Takes a point cloud created from fixture via Plane::pointCloudXYZ()   *
 *      This test is simply looking into the points of the resulting PC.            *
 *                                                                                  *
 *  2. compareTwoResamples:                                                         *
 *      Takes a point cloud created from fixture via Plane::pointCloudXYZ()   *
 *      and checks that two resampled PointNormal clouds created using the same     *
 *      search radius and input cloud match pointwise.                              *
 *                                                                                  *
 *  3. Properties:                                                                  *
 *                                                                                  *
 *      Takes a point cloud created from fixture via Plane::pointCloudXYZ()   *
 *      Checks the resulting PC's properties, i.e., height, width, size, etc.,      *
 *      against the input PC.                                                       *
 *                                                                                  *
 *  4. Magnitude:                                                                   *
 *                                                                                  *
 *      Takes a point cloud created from fixture via Plane::pointCloudXYZ()   *
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
    volcart::shapes::Plane mesh;

    // Note, radius between (0,1] returns an empty PC
    // Additionally, radius less than or equal to 0 will throw error
    // during resampling as defined by PCL
    double radius;

    //init new PC
    pcl::PointCloud<pcl::PointNormal> newCloud;

};

BOOST_FIXTURE_TEST_CASE(PCTest, resampleFix){

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
    for (radius = 2.0; radius <  7.0; radius ++) {

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

}

/**************************************************
 * Check equivalency of two inline resampled PCs  *
 *************************************************/

BOOST_FIXTURE_TEST_CASE(compareTwoResamples, resampleFix){

    //init PC for later resample call
    pcl::PointCloud<pcl::PointNormal> otherCloud;

    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pCloud;

    /*
     * Determine the ideal search radius for the resample call
     */

    float avgDistance = 0;
    int count = 0;

    //This is assuming a quadratic basis for the points
    //For linear, multiply the avgDistance by 1.2
    //Get the avg distance from points in the point cloud
    for (auto a = cloud->begin(); a != cloud->end(); a++){
        for (auto b = cloud->begin(); b != cloud->end(); b++){

            float pDistance = 0;
            if (a != b)
                pDistance = sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));

            count++;
            avgDistance += pDistance;
        }
    }

    avgDistance /= count;

    //set search radius based on 2.5(avgDistance) and call resample()
    radius = 2.5 * avgDistance;

    /*
     * Now, call resample twice on the same input cloud and assign to separate PCs
     */

    //call resample PC twice
    newCloud = volcart::meshing::resamplePointCloud(cloud, radius);
    otherCloud = volcart::meshing::resamplePointCloud(cloud, radius);

    //place resampled points in vectors
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > newCloudData = newCloud.points;
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > otherCloudData = otherCloud.points;

    //Check that the cloud data matches for both resampled clouds
    for (int i = 0; i < newCloudData.size(); i ++) {

        BOOST_CHECK_EQUAL(newCloudData[i].x, otherCloudData[i].x);
        BOOST_CHECK_EQUAL(newCloudData[i].y, otherCloudData[i].y);
        BOOST_CHECK_EQUAL(newCloudData[i].z, otherCloudData[i].z);
        BOOST_CHECK_EQUAL(newCloudData[i].normal_x, otherCloudData[i].normal_x);
        BOOST_CHECK_EQUAL(newCloudData[i].normal_y, otherCloudData[i].normal_y);
        BOOST_CHECK_EQUAL(newCloudData[i].normal_z, otherCloudData[i].normal_z);

    }
}

/*************************************************************************
 * Check equivalency of newly-created PointCloud with a saved PointCloud *
 ************************************************************************/

BOOST_FIXTURE_TEST_CASE(compareSavedResample, resampleFix){

    //init PC for later resample call
    pcl::PointCloud<pcl::PointNormal> convertedSavedCloud;
    pcl::PointCloud<pcl::PointNormal> savedCloud;

    std::cerr << "Reading in resampleExample.pcd" << std::endl;

    //load in the saved PointCloud .pcd file created by resamplePointCloudExample.cpp
    pcl::io::loadPCDFile("resampleExample.pcd", savedCloud);

    //convert savedCloud to PointCloud<pcl::PointNormal>
    //pcl::fromPCLPointCloud2(savedCloud, convertedSavedCloud);

    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pCloud;

    //Determine the ideal search radius for the resample() call
    float avgDistance = 0;
    int count = 0;

    //This is assuming a quadratic basis for the points
    //For linear, multiply the avgDistance by 1.2
    //Get the avg distance from points in the point cloud
    for (auto a = cloud->begin(); a != cloud->end(); a++){
        for (auto b = cloud->begin(); b != cloud->end(); b++){

            float pDistance = 0;
            if (a != b)
                pDistance = sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));

            count++;
            avgDistance += pDistance;
        }
    }

    avgDistance /= count;

    //set search radius based on 2.5(avgDistance) and call resample()
    radius = 2.5 * avgDistance;

    //call resample PC on the test-case-created-PointCloud
    newCloud = volcart::meshing::resamplePointCloud(cloud, radius);


    //place resampled points in vectors
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > newCloudData = newCloud.points;
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > savedCloudData = savedCloud.points;

    //Check that the cloud data matches for both resampled clouds
    for (int i = 0; i < newCloudData.size(); i ++) {

        BOOST_CHECK_EQUAL(newCloudData[i].x, savedCloudData[i].x);
        BOOST_CHECK_EQUAL(newCloudData[i].y, savedCloudData[i].y);
        BOOST_CHECK_EQUAL(newCloudData[i].z, savedCloudData[i].z);
        BOOST_CHECK_EQUAL(newCloudData[i].normal_x, savedCloudData[i].normal_x);
        BOOST_CHECK_EQUAL(newCloudData[i].normal_y, savedCloudData[i].normal_y);
        BOOST_CHECK_EQUAL(newCloudData[i].normal_z, savedCloudData[i].normal_z);
    }
}

/******************************************************
 * Check core properties of original and resampled PC *
 ******************************************************/

BOOST_FIXTURE_TEST_CASE(properties, resampleFix){

    std::cerr << "Testing resampled properties..." << std::endl;
    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pCloud;

    float avgDistance = 0;
    int count = 0;

    //This is assuming a quadratic basis for the points
    //For linear, multiply the avgDistance by 1.2
    //Get the avg distance from points in the point cloud
    for (auto a = cloud->begin(); a != cloud->end(); a++){
        for (auto b = cloud->begin(); b != cloud->end(); b++){

            float pDistance = 0;
            if (a != b)
                pDistance = sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));

            count++;
            avgDistance += pDistance;
        }
    }

    avgDistance /= count;

    //set search radius based on 2.5(avgDistance) and call resample()
    radius = 2.5 * avgDistance;
    std::cerr << "Using search radius of " << radius << std::endl;
    newCloud = volcart::meshing::resamplePointCloud(cloud, radius);

    //if we use the above procedure to set the search radius, it seems the properties in the resampled PC
    //match those of the original. Previously, defining a value of radius and running resulted in failed tests...
    //mainly width() and size().

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