//
// Created by Ryan Taber on 10/28/15.
//

#define BOOST_TEST_MODULE resamplePointCloud

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <pcl/io/pcd_io.h>

#include "common/vc_defines.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Arch.h"
#include "common/shapes/Sphere.h"
#include "common/shapes/Cone.h"
#include "testing/testingUtils.h"
#include "meshing/resamplePointCloud.h"

/************************************************************************************
 *                                                                                  *
 *  resamplePointCloudTest.cpp - tests the functionality of                         *
 *  v-c/meshing/resamplePointCloud.cpp                                              *
 *                                                                                  *
 *                                                                                  *
 *  This file is broken up into a test fixture, resampleFix, which initializes      *
 *  some of the objects used in the fixture test cases:                             *
 *                                                                                  *
 *  1. CompareFixtureAndSavedResampledPlanePointClouds                              *
 *  2. CompareFixtureAndSavedResampledCubePointClouds                               *
 *  3. CompareFixtureAndSavedResampledArchPointClouds                               *
 *  4. CompareFixtureAndSavedResampledSpherePointClouds                             *
 *  5. CompareFixtureAndSavedResampledConePointClouds                               *
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

double CalculateRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

struct PlaneResamplePointCloudFixture {

    PlaneResamplePointCloudFixture() {

        //create a Plane Point Cloud -> convert to pointer ->
        //call resample() and assign results -> save point data from cloud into vector
        _in_PlanePointCloud = _Plane.pointCloudXYZ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr _in_PlaneCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        *_in_PlaneCloudPtr = _in_PlanePointCloud;
        _out_PlanePointCloud = volcart::meshing::resamplePointCloud(_in_PlaneCloudPtr, CalculateRadius(_in_PlaneCloudPtr));
        _out_PlaneCloudPointData = _out_PlanePointCloud.points;

        //Load in saved test file created by resamplePointCloudExample.cpp
        pcl::io::loadPCDFile("PlaneResamplePointCloudExample.pcd", _SavedPlaneCloud);
        _SavedPointCloudData = _SavedPlaneCloud.points;

        std::cerr << "\nsetting up Plane ResamplePCTest objects" << std::endl;
    }

    ~PlaneResamplePointCloudFixture(){ std::cerr << "\ncleaning up Plane ResamplePCTest objects" << std::endl; }

    volcart::shapes::Plane _Plane;
    pcl::PointCloud<pcl::PointXYZ> _in_PlanePointCloud;
    pcl::PointCloud<pcl::PointNormal> _out_PlanePointCloud, _SavedPlaneCloud;
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > _out_PlaneCloudPointData, _SavedPointCloudData;

};


struct CubeResamplePointCloudFixture {

    CubeResamplePointCloudFixture() {

        _in_CubePointCloud = _Cube.pointCloudXYZ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr _in_CubeCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        *_in_CubeCloudPtr = _in_CubePointCloud;
        _out_CubePointCloud = volcart::meshing::resamplePointCloud(_in_CubeCloudPtr, CalculateRadius(_in_CubeCloudPtr));
        _out_CubeCloudPointData = _out_CubePointCloud.points;
        pcl::io::loadPCDFile("CubeResamplePointCloudExample.pcd", _SavedCubeCloud);
        _SavedPointCloudData = _SavedCubeCloud.points;

        std::cerr << "\nsetting up Cube ResamplePCTest objects" << std::endl;
    }

    ~CubeResamplePointCloudFixture(){ std::cerr << "\ncleaning up Cube ResamplePCTest objects" << std::endl; }

    volcart::shapes::Cube _Cube;
    pcl::PointCloud<pcl::PointXYZ> _in_CubePointCloud;
    pcl::PointCloud<pcl::PointNormal> _out_CubePointCloud, _SavedCubeCloud;
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > _out_CubeCloudPointData, _SavedPointCloudData;

};

struct ArchResamplePointCloudFixture {

    ArchResamplePointCloudFixture() {

        _in_ArchPointCloud = _Arch.pointCloudXYZ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr _in_ArchCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        *_in_ArchCloudPtr = _in_ArchPointCloud;
        _out_ArchPointCloud = volcart::meshing::resamplePointCloud(_in_ArchCloudPtr, CalculateRadius(_in_ArchCloudPtr));
        _out_ArchCloudPointData = _out_ArchPointCloud.points;
        pcl::io::loadPCDFile("ArchResamplePointCloudExample.pcd", _SavedArchCloud);
        _SavedPointCloudData = _SavedArchCloud.points;

        std::cerr << "\nsetting up Arch ResamplePCTest objects" << std::endl;
    }

    ~ArchResamplePointCloudFixture(){ std::cerr << "\ncleaning up Arch ResamplePCTest objects" << std::endl; }

    volcart::shapes::Arch _Arch;
    pcl::PointCloud<pcl::PointXYZ> _in_ArchPointCloud;
    pcl::PointCloud<pcl::PointNormal> _out_ArchPointCloud, _SavedArchCloud;
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > _out_ArchCloudPointData, _SavedPointCloudData;

};

struct SphereResamplePointCloudFixture {

    SphereResamplePointCloudFixture() {

        _in_SpherePointCloud = _Sphere.pointCloudXYZ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr _in_SphereCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        *_in_SphereCloudPtr = _in_SpherePointCloud;
        _out_SpherePointCloud = volcart::meshing::resamplePointCloud(_in_SphereCloudPtr, CalculateRadius(_in_SphereCloudPtr));
        _out_SphereCloudPointData = _out_SpherePointCloud.points;
        pcl::io::loadPCDFile("SphereResamplePointCloudExample.pcd", _SavedSphereCloud);
        _SavedPointCloudData = _SavedSphereCloud.points;

        std::cerr << "\nsetting up Sphere ResamplePCTest objects" << std::endl;
    }

    ~SphereResamplePointCloudFixture(){ std::cerr << "\ncleaning up Sphere ResamplePCTest objects" << std::endl; }

    volcart::shapes::Sphere _Sphere;
    pcl::PointCloud<pcl::PointXYZ> _in_SpherePointCloud;
    pcl::PointCloud<pcl::PointNormal> _out_SpherePointCloud, _SavedSphereCloud;
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > _out_SphereCloudPointData, _SavedPointCloudData;

};

struct ConeResamplePointCloudFixture {

    ConeResamplePointCloudFixture() {

        _in_ConePointCloud = _Cone.pointCloudXYZ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr _in_ConeCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        *_in_ConeCloudPtr = _in_ConePointCloud;
        _out_ConePointCloud = volcart::meshing::resamplePointCloud(_in_ConeCloudPtr, CalculateRadius(_in_ConeCloudPtr));
        _out_ConeCloudPointData = _out_ConePointCloud.points;
        pcl::io::loadPCDFile("ConeResamplePointCloudExample.pcd", _SavedConeCloud);
        _SavedPointCloudData = _SavedConeCloud.points;

        std::cerr << "\nsetting up Cone ResamplePCTest objects" << std::endl;
    }

    ~ConeResamplePointCloudFixture(){ std::cerr << "\ncleaning up Cone ResamplePCTest objects" << std::endl; }

    volcart::shapes::Cone _Cone;
    pcl::PointCloud<pcl::PointXYZ> _in_ConePointCloud;
    pcl::PointCloud<pcl::PointNormal> _out_ConePointCloud, _SavedConeCloud;
    std::vector<pcl::PointNormal, Eigen::aligned_allocator<pcl::PointNormal> > _out_ConeCloudPointData, _SavedPointCloudData;

};



/****************************************************************************
 * Check equivalency of Fixture-created PointClouds with a saved PointCloud *
 ****************************************************************************/

BOOST_FIXTURE_TEST_CASE(CompareFixtureAndSavedResampledPlanePointClouds, PlaneResamplePointCloudFixture){

    //Check that the cloud data matches for both resampled clouds
    for (int i = 0; i < _out_PlaneCloudPointData.size(); i ++) {

        volcart::testing::SmallOrClose(_out_PlaneCloudPointData[i].x, _SavedPointCloudData[i].x);
        volcart::testing::SmallOrClose(_out_PlaneCloudPointData[i].y, _SavedPointCloudData[i].y);
        volcart::testing::SmallOrClose(_out_PlaneCloudPointData[i].z, _SavedPointCloudData[i].z);
        volcart::testing::SmallOrClose(_out_PlaneCloudPointData[i].normal_x, _SavedPointCloudData[i].normal_x);
        volcart::testing::SmallOrClose(_out_PlaneCloudPointData[i].normal_y, _SavedPointCloudData[i].normal_y);
        volcart::testing::SmallOrClose(_out_PlaneCloudPointData[i].normal_z, _SavedPointCloudData[i].normal_z);
    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureAndSavedResampledCubePointClouds, CubeResamplePointCloudFixture){

    //Check that the cloud data matches for both resampled clouds
    for (int i = 0; i < _out_CubeCloudPointData.size(); i ++) {

        volcart::testing::SmallOrClose(_out_CubeCloudPointData[i].x, _SavedPointCloudData[i].x);
        volcart::testing::SmallOrClose(_out_CubeCloudPointData[i].y, _SavedPointCloudData[i].y);
        volcart::testing::SmallOrClose(_out_CubeCloudPointData[i].z, _SavedPointCloudData[i].z);
        volcart::testing::SmallOrClose(_out_CubeCloudPointData[i].normal_x, _SavedPointCloudData[i].normal_x);
        volcart::testing::SmallOrClose(_out_CubeCloudPointData[i].normal_y, _SavedPointCloudData[i].normal_y);
        volcart::testing::SmallOrClose(_out_CubeCloudPointData[i].normal_z, _SavedPointCloudData[i].normal_z);
    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureAndSavedResampledArchPointClouds, ArchResamplePointCloudFixture){

    //Check that the cloud data matches for both resampled clouds
    for (int i = 0; i < _out_ArchCloudPointData.size(); i ++) {

        volcart::testing::SmallOrClose(_out_ArchCloudPointData[i].x, _SavedPointCloudData[i].x);
        volcart::testing::SmallOrClose(_out_ArchCloudPointData[i].y, _SavedPointCloudData[i].y);
        volcart::testing::SmallOrClose(_out_ArchCloudPointData[i].z, _SavedPointCloudData[i].z);
        volcart::testing::SmallOrClose(_out_ArchCloudPointData[i].normal_x, _SavedPointCloudData[i].normal_x);
        volcart::testing::SmallOrClose(_out_ArchCloudPointData[i].normal_y, _SavedPointCloudData[i].normal_y);
        volcart::testing::SmallOrClose(_out_ArchCloudPointData[i].normal_z, _SavedPointCloudData[i].normal_z);
    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureAndSavedResampledSpherePointClouds, SphereResamplePointCloudFixture){

    //Check that the cloud data matches for both resampled clouds
    for (int i = 0; i < _out_SphereCloudPointData.size(); i ++) {

        volcart::testing::SmallOrClose(_out_SphereCloudPointData[i].x, _SavedPointCloudData[i].x);
        volcart::testing::SmallOrClose(_out_SphereCloudPointData[i].y, _SavedPointCloudData[i].y);
        volcart::testing::SmallOrClose(_out_SphereCloudPointData[i].z, _SavedPointCloudData[i].z);
        volcart::testing::SmallOrClose(_out_SphereCloudPointData[i].normal_x, _SavedPointCloudData[i].normal_x);
        volcart::testing::SmallOrClose(_out_SphereCloudPointData[i].normal_y, _SavedPointCloudData[i].normal_y);
        volcart::testing::SmallOrClose(_out_SphereCloudPointData[i].normal_z, _SavedPointCloudData[i].normal_z);
    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureAndSavedResampledConePointClouds, ConeResamplePointCloudFixture){

    //Check that the cloud data matches for both resampled clouds
    for (int i = 0; i < _out_ConeCloudPointData.size(); i ++) {

        volcart::testing::SmallOrClose(_out_ConeCloudPointData[i].x, _SavedPointCloudData[i].x);
        volcart::testing::SmallOrClose(_out_ConeCloudPointData[i].y, _SavedPointCloudData[i].y);
        volcart::testing::SmallOrClose(_out_ConeCloudPointData[i].z, _SavedPointCloudData[i].z);
        volcart::testing::SmallOrClose(_out_ConeCloudPointData[i].normal_x, _SavedPointCloudData[i].normal_x);
        volcart::testing::SmallOrClose(_out_ConeCloudPointData[i].normal_y, _SavedPointCloudData[i].normal_y);
        volcart::testing::SmallOrClose(_out_ConeCloudPointData[i].normal_z, _SavedPointCloudData[i].normal_z);
    }
}




/*
 * helper function
 *
 */
double CalculateRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    float avgDistance = 0;
    int count = 0;

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

    return 2.5 * avgDistance;
}
