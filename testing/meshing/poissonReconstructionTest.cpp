//
// Created by Ryan Taber on 11/2/15.
//


#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE resamplePointCloud

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "poissonReconstruction.h"

struct poissonFix {

    poissonFix() {

        pCloud = mesh.pointCloudNormal();
        std::cerr << "\nsetting up poissonReconstructionTest objects" << std::endl;
    }

    ~poissonFix(){ std::cerr << "\ncleaning up poissonReconstructionTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointNormal> pCloud;
    volcart::testing::testingMesh mesh;
    pcl::PolygonMesh polyMesh;

};

BOOST_FIXTURE_TEST_CASE(poissonTest, poissonFix){

    //convert pCloud to Ptr for poisson() call
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
    *cloud = pCloud;

    BOOST_CHECK(true);

}

