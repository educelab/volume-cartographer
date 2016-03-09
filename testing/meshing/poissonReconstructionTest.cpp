//
// Created by Ryan Taber on 11/2/15.
//


#ifndef VC_PREBUILT_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE poissonReconstruction

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "shapes.h"
#include "poissonReconstruction.h"
#include <pcl/conversions.h>
#include <pcl/io/obj_io.h>


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



struct PlanePoissonReconstructionFixture {

    PlanePoissonReconstructionFixture() {

        //fill point clouds
        _in_PlaneCloud = _Plane.pointCloudNormal();

        //init pointers to point clouds
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_PlaneCloudPtr(new pcl::PointCloud<pcl::PointNormal>);

        //assign ptrs
        *_in_PlaneCloudPtr = _in_PlaneCloud;

        //make call to poissonReconstruction() and assign results to polygon mesh
        _out_PlanePolygonMesh = volcart::meshing::poissonReconstruction(_in_PlaneCloudPtr);

        //load data from saved files
        pcl::io::loadOBJFile("PlanePoissonReconstruction.obj", _SavedPlaneSurface);

        std::cerr << "\nsetting up PlanePoissonReconstruction objects" << std::endl;
    }

    ~PlanePoissonReconstructionFixture(){ std::cerr << "\ncleaning up PlanePoissonReconstructionTest objects" << std::endl; }

    volcart::shapes::Plane _Plane;
    pcl::PointCloud<pcl::PointNormal> _in_PlaneCloud;
    pcl::PolygonMesh _out_PlanePolygonMesh, _SavedPlaneSurface;

};

struct CubePoissonReconstructionFixture {

    CubePoissonReconstructionFixture() {

        //fill point cloud
        _in_CubeCloud = _Cube.pointCloudNormal();

        //init pointer to point cloud
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_CubeCloudPtr(new pcl::PointCloud<pcl::PointNormal>);

        //assign ptr
        *_in_CubeCloudPtr = _in_CubeCloud;

        //make call to poissonReconstruction() and assign results to polygon mesh
        _out_CubePolygonMesh = volcart::meshing::poissonReconstruction(_in_CubeCloudPtr);

        //load data from saved files
        pcl::io::loadOBJFile("CubePoissonReconstruction.obj", _SavedCubeSurface);

        std::cerr << "\nsetting up poissonReconstructionTest objects" << std::endl;
    }

    ~CubePoissonReconstructionFixture(){ std::cerr << "\ncleaning up poissonReconstructionTest objects" << std::endl; }

    //init shapes
    volcart::shapes::Cube _Cube;
    pcl::PointCloud<pcl::PointNormal> _in_CubeCloud;
    pcl::PolygonMesh _out_CubePolygonMesh, _SavedCubeSurface;

};


struct ArchPoissonReconstructionFixture {

    ArchPoissonReconstructionFixture() {

        //fill point cloud
        _in_ArchCloud = _Arch.pointCloudNormal();

        //init pointer to point cloud
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_ArchCloudPtr(new pcl::PointCloud<pcl::PointNormal>);

        //assign ptr
        *_in_ArchCloudPtr = _in_ArchCloud;

        //make call to poissonReconstruction() and assign results to polygon mesh
        _out_ArchPolygonMesh = volcart::meshing::poissonReconstruction(_in_ArchCloudPtr);

        //load data from saved files
        pcl::io::loadOBJFile("ArchPoissonReconstruction.obj", _SavedArchSurface);

        std::cerr << "\nsetting up ArchPoissonReconstruction objects" << std::endl;
    }

    ~ArchPoissonReconstructionFixture(){ std::cerr << "\ncleaning up ArchPoissonReconstruction objects" << std::endl; }

    volcart::shapes::Arch _Arch;
    pcl::PointCloud<pcl::PointNormal> _in_ArchCloud;
    pcl::PolygonMesh _out_ArchPolygonMesh, _SavedArchSurface;

};

struct SpherePoissonReconstructionFixture {

    SpherePoissonReconstructionFixture() {

        //fill point cloud
        _in_SphereCloud = _Sphere.pointCloudNormal();

        //init pointer to point cloud
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_SphereCloudPtr(new pcl::PointCloud<pcl::PointNormal>);

        //assign ptr
        *_in_SphereCloudPtr = _in_SphereCloud;

        //make call to poissonReconstruction() and assign results to polygon mesh
        _out_SpherePolygonMesh = volcart::meshing::poissonReconstruction(_in_SphereCloudPtr);

        //load data from saved files
        pcl::io::loadOBJFile("SpherePoissonReconstruction.obj", _SavedSphereSurface);

        std::cerr << "\nsetting up SpherePoissonReconstruction objects" << std::endl;
    }

    ~SpherePoissonReconstructionFixture(){ std::cerr << "\ncleaning up SpherePoissonReconstruction objects" << std::endl; }

    volcart::shapes::Sphere _Sphere;
    pcl::PointCloud<pcl::PointNormal> _in_SphereCloud;
    pcl::PolygonMesh _out_SpherePolygonMesh, _SavedSphereSurface;

};

struct ConePoissonReconstructionFixture {

    ConePoissonReconstructionFixture() {

        //fill point cloud
        _in_ConeCloud = _Cone.pointCloudNormal();

        //init pointer to point cloud
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_ConeCloudPtr(new pcl::PointCloud<pcl::PointNormal>);

        //assign ptr
        *_in_ConeCloudPtr = _in_ConeCloud;

        //make call to poissonReconstruction() and assign results to polygon mesh
        _out_ConePolygonMesh = volcart::meshing::poissonReconstruction(_in_ConeCloudPtr);

        //load data from saved file
        pcl::io::loadOBJFile("ConePoissonReconstruction.obj", _SavedConeSurface);

        std::cerr << "\nsetting up ConePoissonReconstruction objects" << std::endl;
    }

    ~ConePoissonReconstructionFixture(){ std::cerr << "\ncleaning up ConePoissonReconstruction objects" << std::endl; }

    volcart::shapes::Cone _Cone;
    pcl::PointCloud<pcl::PointNormal> _in_ConeCloud;
    pcl::PolygonMesh _out_ConePolygonMesh, _SavedConeSurface;


};

BOOST_FIXTURE_TEST_CASE(ComparePlanePoissonReonstructionWithSavedPlaneFile, PlanePoissonReconstructionFixture){

    //check number of polys and cloud.data sizes for each poissonRecon mesh
    BOOST_CHECK_EQUAL(_out_PlanePolygonMesh.polygons.size(), _SavedPlaneSurface.polygons.size());

    for (int p = 0; p < _SavedPlaneSurface.polygons.size(); p++) {
        for (int v = 0; v < _SavedPlaneSurface.polygons[p].vertices.size(); v++) {

            BOOST_CHECK_EQUAL(_SavedPlaneSurface.polygons[p].vertices[v], _out_PlanePolygonMesh.polygons[p].vertices[v]);
        }
    }

    //check the converted clouds from both meshes to check for equality

    pcl::PointCloud<pcl::PointNormal> out_PlaneMeshConvertedToCloud, SavedPlaneSurfaceConvertedToCloud ;
    pcl::fromPCLPointCloud2(_out_PlanePolygonMesh.cloud, out_PlaneMeshConvertedToCloud);
    pcl::fromPCLPointCloud2(_SavedPlaneSurface.cloud, SavedPlaneSurfaceConvertedToCloud);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(out_PlaneMeshConvertedToCloud.size(), SavedPlaneSurfaceConvertedToCloud.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < out_PlaneMeshConvertedToCloud.points.size(); i++ ){
        for (int j = 0; j < 3; j ++) {

            BOOST_CHECK_CLOSE(out_PlaneMeshConvertedToCloud.points[i].data[j],
                              SavedPlaneSurfaceConvertedToCloud.points[i].data[j], 0.01);
        }
    }

}

BOOST_FIXTURE_TEST_CASE(CompareCubePoissonReonstructionWithSavedCubeFile, CubePoissonReconstructionFixture){

    //check number of polys and cloud.data sizes for each poissonRecon mesh
    BOOST_CHECK_EQUAL(_out_CubePolygonMesh.polygons.size(), _SavedCubeSurface.polygons.size());

    for (int p = 0; p < _SavedCubeSurface.polygons.size(); p++) {
        for (int v = 0; v < _SavedCubeSurface.polygons[p].vertices.size(); v++) {

            BOOST_CHECK_EQUAL(_SavedCubeSurface.polygons[p].vertices[v], _out_CubePolygonMesh.polygons[p].vertices[v]);
        }
    }

    //check the converted clouds from both meshes to check for equality

    pcl::PointCloud<pcl::PointNormal> out_CubeMeshConvertedToCloud, SavedCubeSurfaceConvertedToCloud ;
    pcl::fromPCLPointCloud2(_out_CubePolygonMesh.cloud, out_CubeMeshConvertedToCloud);
    pcl::fromPCLPointCloud2(_SavedCubeSurface.cloud, SavedCubeSurfaceConvertedToCloud);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(out_CubeMeshConvertedToCloud.size(), SavedCubeSurfaceConvertedToCloud.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < out_CubeMeshConvertedToCloud.points.size(); i++ ){
        for (int j = 0; j < 3; j ++) {

            BOOST_CHECK_CLOSE(out_CubeMeshConvertedToCloud.points[i].data[j],
                              SavedCubeSurfaceConvertedToCloud.points[i].data[j], 0.01);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(CompareArchPoissonReonstructionWithSavedArchFile, ArchPoissonReconstructionFixture){

    //check number of polys and cloud.data sizes for each poissonRecon mesh
    BOOST_CHECK_EQUAL(_out_ArchPolygonMesh.polygons.size(), _SavedArchSurface.polygons.size());

    for (int p = 0; p < _SavedArchSurface.polygons.size(); p++) {
        for (int v = 0; v < _SavedArchSurface.polygons[p].vertices.size(); v++) {

            BOOST_CHECK_EQUAL(_SavedArchSurface.polygons[p].vertices[v], _out_ArchPolygonMesh.polygons[p].vertices[v]);
        }
    }

    //check the converted clouds from both meshes to check for equality

    pcl::PointCloud<pcl::PointNormal> out_ArchMeshConvertedToCloud, SavedArchSurfaceConvertedToCloud ;
    pcl::fromPCLPointCloud2(_out_ArchPolygonMesh.cloud, out_ArchMeshConvertedToCloud);
    pcl::fromPCLPointCloud2(_SavedArchSurface.cloud, SavedArchSurfaceConvertedToCloud);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(out_ArchMeshConvertedToCloud.size(), SavedArchSurfaceConvertedToCloud.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < out_ArchMeshConvertedToCloud.points.size(); i++ ){
        for (int j = 0; j < 3; j ++) {

            BOOST_CHECK_CLOSE(out_ArchMeshConvertedToCloud.points[i].data[j],
                              SavedArchSurfaceConvertedToCloud.points[i].data[j], 0.01);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSpherePoissonReonstructionWithSavedSphereFile, SpherePoissonReconstructionFixture){

    //check number of polys and cloud.data sizes for each poissonRecon mesh
    BOOST_CHECK_EQUAL(_out_SpherePolygonMesh.polygons.size(), _SavedSphereSurface.polygons.size());

    for (int p = 0; p < _SavedSphereSurface.polygons.size(); p++) {
        for (int v = 0; v < _SavedSphereSurface.polygons[p].vertices.size(); v++) {

            BOOST_CHECK_EQUAL(_SavedSphereSurface.polygons[p].vertices[v], _out_SpherePolygonMesh.polygons[p].vertices[v]);
        }
    }

    //check the converted clouds from both meshes to check for equality

    pcl::PointCloud<pcl::PointNormal> out_SphereMeshConvertedToCloud, SavedSphereSurfaceConvertedToCloud ;
    pcl::fromPCLPointCloud2(_out_SpherePolygonMesh.cloud, out_SphereMeshConvertedToCloud);
    pcl::fromPCLPointCloud2(_SavedSphereSurface.cloud, SavedSphereSurfaceConvertedToCloud);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(out_SphereMeshConvertedToCloud.size(), SavedSphereSurfaceConvertedToCloud.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < out_SphereMeshConvertedToCloud.points.size(); i++ ){
        for (int j = 0; j < 3; j ++) {

            BOOST_CHECK_CLOSE(out_SphereMeshConvertedToCloud.points[i].data[j],
                              SavedSphereSurfaceConvertedToCloud.points[i].data[j], 0.01);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(CompareConePoissonReonstructionWithSavedConeFile, ConePoissonReconstructionFixture){

    //check number of polys and cloud.data sizes for each poissonRecon mesh
    BOOST_CHECK_EQUAL(_out_ConePolygonMesh.polygons.size(), _SavedConeSurface.polygons.size());

    for (int p = 0; p < _SavedConeSurface.polygons.size(); p++) {
        for (int v = 0; v < _SavedConeSurface.polygons[p].vertices.size(); v++) {

            BOOST_CHECK_EQUAL(_SavedConeSurface.polygons[p].vertices[v], _out_ConePolygonMesh.polygons[p].vertices[v]);
        }
    }

    //check the converted clouds from both meshes to check for equality

    pcl::PointCloud<pcl::PointNormal> out_ConeMeshConvertedToCloud, SavedConeSurfaceConvertedToCloud ;
    pcl::fromPCLPointCloud2(_out_ConePolygonMesh.cloud, out_ConeMeshConvertedToCloud);
    pcl::fromPCLPointCloud2(_SavedConeSurface.cloud, SavedConeSurfaceConvertedToCloud);

    //check the sizes of the new point clouds for equality
    BOOST_CHECK_EQUAL(out_ConeMeshConvertedToCloud.size(), SavedConeSurfaceConvertedToCloud.size());

    //now check the points in each of the converted clouds for equality
    for (int i = 0; i < out_ConeMeshConvertedToCloud.points.size(); i++ ){
        for (int j = 0; j < 3; j ++) {

            BOOST_CHECK_CLOSE(out_ConeMeshConvertedToCloud.points[i].data[j],
                              SavedConeSurfaceConvertedToCloud.points[i].data[j], 0.01);
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
