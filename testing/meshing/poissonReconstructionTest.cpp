//
// Created by Ryan Taber on 11/2/15.
//


#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE poissonReconstruction

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "poissonReconstruction.h"
#include "pcl/conversions.h"
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>


/************************************************************************************
 *                                                                                  *
 *  poissonReconstructionTest.cpp - tests the functionality of                      *
 *  v-c/meshing/poissonReconstruction.cpp with the ultimate goal of the following:  *
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
 *  2. fromFileSurfaceComparison (fixture test case)
 *  3. surfaceComparison (fixture test case)                                        *
 *  4. emptyCloud (auto test case)                                                  *
 *  5. onePoint (auto test case)                                                    *
 *  6. moreThanOnePoint (auto test case)                                            *
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

/*
 * Only difference here is that we call poissonRecon within the file and save it away
 * for later comparison within fromFileSurfaceComparison() test case
 */

struct savedPoissonFix {

    savedPoissonFix() {

        pCloud = mesh.pointCloudNormal();
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
        *cloud = pCloud;
        polyMesh = volcart::meshing::poissonReconstruction(cloud);

        // Write polygons to file
        pcl::io::saveOBJFile( "poissonSurface.obj", polyMesh);

        // Write cloud data to file
        pcl::io::savePCDFile("poissonSurface.pcd", polyMesh.cloud);

        std::cerr << "\nsetting up savedPoissonReconstructionTest objects" << std::endl;
    }

    ~savedPoissonFix(){ std::cerr << "\ncleaning up savedPoissonReconstructionTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointNormal> pCloud;
    volcart::testing::testingMesh mesh;
    pcl::PolygonMesh polyMesh;

};
/*
 * Not so much a test as it is checking out the polys of the resulting
 * poissonRecon mesh from an input cloud
 */

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
        BOOST_CHECK_GT(polyMesh.cloud.data.size(), 0);
    }

    /*Printing out vertices*/
//    //Check the polygons of the new mesh
//    for (int i = 0; i < polyMesh.polygons.size(); i++){
//        std::cout << "Poly " << i << ": ";
//        for (int j = 0; j < polyMesh.polygons[i].vertices.size(); j++) {
//
//            std::cout << polyMesh.polygons[i].vertices[j] << " | ";
//        }
//        std::cout << "\n";
//    }

}

/*
   Goal of this test is to confirm that the same surface is reconstructed given
   the same input point cloud. One surface is created with poissonReconstruction()
   and then compared to a saved OBJ file representing a surface called from the
   same input point cloud.

*/

BOOST_FIXTURE_TEST_CASE(fromFileSurfaceComparison, savedPoissonFix){

    pcl::PolygonMesh otherPoly;


    // Load in polygonMesh saved from fixture
    pcl::PolygonMesh savedSurface;
    pcl::io::loadOBJFile("poissonSurface.obj", savedSurface );

    // Load pcd file points
    pcl::PCLPointCloud2 savedPoints;
    pcl::io::loadPCDFile("poissonSurface.pcd", savedPoints);

    //convert pCloud to Ptr for poisson() call
    pcl::PointCloud<pcl::PointNormal>::Ptr testCloud(new pcl::PointCloud<pcl::PointNormal>);
    *testCloud = pCloud;

    //call recon and assign
    otherPoly = volcart::meshing::poissonReconstruction(testCloud);

    /* First, let's check the values of the polygon vertices in each of the PolygonMesh objects*/

    //check number of polys and cloud.data sizes for each poissonRecon mesh
    BOOST_CHECK_EQUAL(savedSurface.polygons.size(), otherPoly.polygons.size());

    for (int p = 0; p < savedSurface.polygons.size(); p++) {
        for (int v = 0; v < savedSurface.polygons[p].vertices.size(); v++) {

            BOOST_CHECK_EQUAL(savedSurface.polygons[p].vertices[v], otherPoly.polygons[p].vertices[v]);
        }
    }

    /* Now, let's check the converted clouds from both meshes to check for equality */

    //convert poly meshes clouds to PC<PointNormal>
    pcl::PointCloud<pcl::PointNormal> convCloud1, convCloud2 ;
    pcl::fromPCLPointCloud2(otherPoly.cloud, convCloud1);
    pcl::fromPCLPointCloud2(savedPoints, convCloud2);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(convCloud1.size(), convCloud2.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < convCloud1.points.size(); i++ ){
        //std::cout << "Point " << i << ": " << std::endl;
        for (int j = 0; j < 3; j ++) {

            BOOST_CHECK_CLOSE(convCloud1.points[i].data[j], convCloud2.points[i].data[j], 0.01);
        }
    }

}

/*
   Goal of this test is to confirm that the same surface is reconstructed given
   the same input point cloud (within the test)
*/

BOOST_FIXTURE_TEST_CASE(surfaceComparison, poissonFix){

    pcl::PolygonMesh otherPoly;

    //convert pCloud to Ptr for poisson() call
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    *cloud = pCloud;

    //call recon and assign
    polyMesh = volcart::meshing::poissonReconstruction(cloud);
    otherPoly = volcart::meshing::poissonReconstruction(cloud);

    /* First, let's check the values of the polygon vertices in each of the PolygonMesh objects*/

    //check number of polys and cloud.data sizes for each poissonRecon mesh
    BOOST_CHECK_EQUAL(polyMesh.polygons.size(), otherPoly.polygons.size());
    BOOST_CHECK_EQUAL(polyMesh.cloud.data.size(), otherPoly.cloud.data.size());

    for (int p = 0; p < polyMesh.polygons.size(); p++) {
        for (int v = 0; v < polyMesh.polygons[p].vertices.size(); v++) {

            BOOST_CHECK_EQUAL(polyMesh.polygons[p].vertices[v], otherPoly.polygons[p].vertices[v]);
        }
    }

    /* Now, let's check the converted clouds inside the PolygonMesh to check for equality */

    //convert poly meshes clouds to PC<PointNormal>
    pcl::PointCloud<pcl::PointNormal> convCloud1, convCloud2 ;
    pcl::fromPCLPointCloud2(polyMesh.cloud, convCloud1);
    pcl::fromPCLPointCloud2(otherPoly.cloud, convCloud2);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(convCloud1.size(), convCloud2.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < convCloud1.points.size(); i++ ){
        //std::cout << "Point " << i << ": " << std::endl;
        for (int j = 0; j < 3; j ++) {
            //std::cout << convCloud1.points[i].data[j] << " || " << convCloud2.points[i].data[j] << std::endl;

            BOOST_CHECK_EQUAL(convCloud1.points[i].data[j], convCloud2.points[i].data[j]);
        }
    }

}


/*
 * emptyCloud, onePoint, moreThanOnePoint are all simple tests that check the returned PolygonMesh
 * polygon and cloud.data sizes against expected values based on the poissonRecon algorithm.
 * These auto test cases do not rely on poissonFix for
 */

BOOST_AUTO_TEST_CASE(emptyCloud){

    //create a new PointCloud::Ptr object
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PolygonMesh mesh;

    //reconstruct surface with poissonRecon()
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

    //Place a point in the cloud
    pcl::PointNormal point;
    point.x = 1;
    point.y = 1;
    point.z = 1;
    point.normal_x = 0;
    point.normal_y = 1;
    point.normal_z = 0;
    cloud->push_back(point);

    //call poissonReconstructin()
    pcl::PolygonMesh mesh;
    mesh = volcart::meshing::poissonReconstruction(cloud);

    //check to see how the poissonRecon handles a cloud with only one point
    if (cloud->empty()) {
        //cloud shouldn't be empty
        BOOST_CHECK(false);

    } else if (cloud->size() == 1){
        BOOST_CHECK_EQUAL(mesh.polygons.size(), 0);
        BOOST_CHECK_EQUAL(mesh.cloud.data.size(), 0);
    }else {
        //throw false test if cloud size > 1
        BOOST_CHECK(false);
    }

}

BOOST_AUTO_TEST_CASE(moreThanOnePoint) {

    //create a new PointCloud::Ptr object and fill with 2 points
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);

    //Place a point in the cloud
    pcl::PointNormal point;
    for (int i = 0; i < 2; i++) {
        point.x = i;
        point.y = i;
        point.z = i;
        point.normal_x = 0;
        point.normal_y = 1;
        point.normal_z = 0;

        cloud->push_back(point);
    }

    pcl::PolygonMesh mesh;
    mesh = volcart::meshing::poissonReconstruction(cloud);

    //check to see how the poissonRecon handles a cloud with multiple points
    if (cloud->empty() || cloud->size() == 1) {
        //cloud shouldn't be empty or have one point
        BOOST_CHECK(false);

    } else if (cloud->size() == 2){
        BOOST_CHECK_GT(mesh.polygons.size(), 0);
        BOOST_CHECK_GT(mesh.cloud.data.size(), 0);
    }else {
        //throw false test if cloud size > 1
        BOOST_CHECK(false);
    }

}
