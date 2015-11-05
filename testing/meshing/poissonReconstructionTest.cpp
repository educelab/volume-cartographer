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
#include "pcl/conversions.h"

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

BOOST_FIXTURE_TEST_CASE(poissonTest, poissonFix) {

    //convert pCloud to Ptr for poisson() call
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    *cloud = pCloud;

    polyMesh = volcart::meshing::poissonReconstruction(cloud);

    //check to see that there are actually polygons if the cloud wasn't empty
    if (cloud->empty()){

        //should be empty poly set
        BOOST_CHECK_EQUAL(polyMesh.polygons.size(), 0);
    }else{

        BOOST_CHECK_GT(polyMesh.polygons.size(), 0);
    }

    //Check the polygons of the new mesh
    for (int i = 0; i < polyMesh.polygons.size(); i++){
        std::cout << "Poly " << i << ": ";
        for (int j = 0; j < polyMesh.polygons[i].vertices.size(); j++) {

            std::cout << polyMesh.polygons[i].vertices[j] << " | ";


        }
        std::cout << "\n";
    }


    BOOST_CHECK(true);

}

/*
   Goal of this test is to confirm that the same surface is reconstructed given
   the same input point cloud
*/

BOOST_FIXTURE_TEST_CASE(cloudComparison, poissonFix){

    pcl::PolygonMesh otherPoly;

    //convert pCloud to Ptr for poisson() call
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    *cloud = pCloud;

    //call recon and assign
    polyMesh = volcart::meshing::poissonReconstruction(cloud);
    otherPoly = volcart::meshing::poissonReconstruction(cloud);


    //convert poly meshes clouds to PC<PointNormal>
    pcl::PointCloud<pcl::PointNormal> convCloud1, convCloud2 ;
    pcl::fromPCLPointCloud2(polyMesh.cloud, convCloud1);
    pcl::fromPCLPointCloud2(otherPoly.cloud, convCloud2);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(convCloud1.size(), convCloud2.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < convCloud1.points.size(); i++ ){
        std::cout << "Point " << i << ": " << std::endl;
        for (int j = 0; j < 3; j ++) {
            std::cout << convCloud1.points[i].data[j] << " || " << convCloud2.points[i].data[j] << std::endl;

            BOOST_CHECK_EQUAL(convCloud1.points[i].data[j], convCloud2.points[i].data[j]);
        }
    }




}

BOOST_AUTO_TEST_CASE(emptyCloud){

    //create a new PointCloud::Ptr object
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

    pcl::PolygonMesh mesh;

    mesh = volcart::meshing::poissonReconstruction(cloud);

    //check to see how the poissonRecon handles an empty cloud
    if (cloud->empty()) {
        //check polys and cloud data
        BOOST_CHECK_EQUAL(mesh.polygons.size(), 0);
        BOOST_CHECK_EQUAL(mesh.cloud.data.size(), 0);
    } else {
        //throw false test
        BOOST_CHECK(false);
    }
}

BOOST_AUTO_TEST_CASE(onePoint) {

    //create a new PointCloud::Ptr object
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointNormal point;
    point.x = 1;
    point.y = 1;
    point.z = 1;

    cloud->push_back(point);

    pcl::PolygonMesh mesh;

    mesh = volcart::meshing::poissonReconstruction(cloud);

    //check to see how the poissonRecon handles a cloud with only one point
    if (cloud->empty()) {
        //cloud shouldn't be empty
        BOOST_CHECK(false);

    } else if (cloud->size() == 1){
        BOOST_CHECK_EQUAL(mesh.polygons.size(), 1);
        BOOST_CHECK_EQUAL(mesh.cloud.data.size(), 1);
    }else {
        //throw false test if cloud size > 1
        BOOST_CHECK(false);
    }

}
