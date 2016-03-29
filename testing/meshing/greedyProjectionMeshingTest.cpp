//
// Created by Melissa Shankle on 10/26/15.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE greedyProjectionMeshing

#include <boost/test/unit_test.hpp>
#include "vc_defines.h"
#include "shapes.h"
#include "greedyProjectionMeshing.h"
#include <pcl/io/obj_io.h>
#include <pcl/conversions.h>


/****************************************************************************************
 *                                                                                      *
 *  greedyProjectionMeshingTest.cpp -  tests the functionality of changes to            *
 *      /v-c/meshing/greedyProjectionMeshing.cpp                                        *
 *  The ultimate goal of this file is the following:                                    *
 *                                                                                      *
 *      1. Check whether changes to greedyProjectionMeshing.cpp create a                *
 *         correct mesh based on a previously created, correct mesh. This is performed  *
 *         via the compareMeshes test case.                                             *
 *                                                                                      *
 *  Test Cases:                                                                         *
 *  1. CompareSavedAndFixturePlaneGreedyProjections                                     *
 *  2. CompareSavedAndFixtureCubeGreedyProjections                                      *
 *  3. CompareSavedAndFixtureArchGreedyProjections                                      *
 *  4. CompareSavedAndFixtureSphereGreedyProjections                                    *
 *  5. CompareSavedAndFixtureConeGreedyProjections                                      *
 *                                                                                      *
 *  Input:                                                                              *
 *      - OBJ file to be loaded in as a PCL::PolygonMesh                                *
 *      - input file created by greedyProjection example file                           *
 *                                                                                      *
 *  Test-Specific Output:                                                               *
 *      Either a message stating the meshes are the same or error messages according    *
 *      to Boost.                                                                       *
 *                                                                                      *
 *  Miscellaneous:                                                                      *
 *      See the /testing/meshing wiki for more information on this test.                *
 *                                                                                      *
 ***************************************************************************************/

// General outline for test
//   Create new mesh using greedyProjectionMeshing
//   Read in input for test comparison into PolygonMesh
//   Convert both meshes to correct types
//   Compare new mesh with known mesh for equivalency
//   If errors occur, output them. Otherwise give success message.


struct PlaneGreedyProjectionFixture {

    PlaneGreedyProjectionFixture() {

        std::cerr << "Setting up greedyProjectionMeshing Plane objects." << std::endl;

        // Create a mesh
         _in_PlanePointNormalCloud = _Plane.pointCloudNormal();

        // Create pointer for the mesh to pass to greedyProjectionMeshing
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_PlanePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
        *_in_PlanePointNormalCloudPtr = _in_PlanePointNormalCloud;

        // Call greedyProjectionMeshing() and store results in PolygonMesh
        _out_FixturePlanePolygonMesh = volcart::meshing::greedyProjectionMeshing(_in_PlanePointNormalCloudPtr, 100, 2.0, 2.5);

        // Load in old mesh for comparison in the compareMeshes test case below
        pcl::io::loadOBJFile("PlaneGreedyProjectionMeshing.obj", _in_SavedPlanePolygonMesh);

        pcl::fromPCLPointCloud2(_out_FixturePlanePolygonMesh.cloud, _out_FixturePlanePointCloud);
        pcl::fromPCLPointCloud2(_in_SavedPlanePolygonMesh.cloud, _out_SavedPlanePointCloud);
    }

    ~PlaneGreedyProjectionFixture(){ std::cerr << "Cleaning up greedyProjectionMeshing Plane objects" << std::endl; }

    volcart::shapes::Plane _Plane;
    pcl::PointCloud<pcl::PointNormal> _in_PlanePointNormalCloud;
    pcl::PolygonMesh _out_FixturePlanePolygonMesh, _in_SavedPlanePolygonMesh;
    // point clouds to be filled for comparison in test case
    pcl::PointCloud<pcl::PointNormal> _out_FixturePlanePointCloud, _out_SavedPlanePointCloud;

};

struct CubeGreedyProjectionFixture {

    CubeGreedyProjectionFixture() {

        std::cerr << "Setting up greedyProjectionMeshing Cube objects." << std::endl;

        _in_CubePointNormalCloud = _Cube.pointCloudNormal();
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_CubePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
        *_in_CubePointNormalCloudPtr = _in_CubePointNormalCloud;
        _out_FixtureCubePolygonMesh = volcart::meshing::greedyProjectionMeshing(_in_CubePointNormalCloudPtr, 100, 2.0, 2.5);
        pcl::io::loadOBJFile("CubeGreedyProjectionMeshing.obj", _in_SavedCubePolygonMesh);
        pcl::fromPCLPointCloud2(_out_FixtureCubePolygonMesh.cloud, _out_FixtureCubePointCloud);
        pcl::fromPCLPointCloud2(_in_SavedCubePolygonMesh.cloud, _out_SavedCubePointCloud);
    }

    ~CubeGreedyProjectionFixture(){ std::cerr << "Cleaning up greedyProjectionMeshing Cube objects" << std::endl; }

    volcart::shapes::Cube _Cube;
    pcl::PointCloud<pcl::PointNormal> _in_CubePointNormalCloud;
    pcl::PolygonMesh _out_FixtureCubePolygonMesh, _in_SavedCubePolygonMesh;
    pcl::PointCloud<pcl::PointNormal> _out_FixtureCubePointCloud, _out_SavedCubePointCloud;

};

struct ArchGreedyProjectionFixture {

    ArchGreedyProjectionFixture() {

        std::cerr << "Setting up greedyProjectionMeshing Arch objects." << std::endl;

        _in_ArchPointNormalCloud = _Arch.pointCloudNormal();
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_ArchPointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
        *_in_ArchPointNormalCloudPtr = _in_ArchPointNormalCloud;
        _out_FixtureArchPolygonMesh = volcart::meshing::greedyProjectionMeshing(_in_ArchPointNormalCloudPtr, 100, 2.0, 2.5);
        pcl::io::loadOBJFile("ArchGreedyProjectionMeshing.obj", _in_SavedArchPolygonMesh);
        pcl::fromPCLPointCloud2(_out_FixtureArchPolygonMesh.cloud, _out_FixtureArchPointCloud);
        pcl::fromPCLPointCloud2(_in_SavedArchPolygonMesh.cloud, _out_SavedArchPointCloud);
    }

    ~ArchGreedyProjectionFixture(){ std::cerr << "Cleaning up greedyProjectionMeshing Arch objects" << std::endl; }

    volcart::shapes::Arch _Arch;
    pcl::PointCloud<pcl::PointNormal> _in_ArchPointNormalCloud;
    pcl::PolygonMesh _out_FixtureArchPolygonMesh, _in_SavedArchPolygonMesh;
    pcl::PointCloud<pcl::PointNormal> _out_FixtureArchPointCloud, _out_SavedArchPointCloud;

};

struct SphereGreedyProjectionFixture {

    SphereGreedyProjectionFixture() {

        std::cerr << "Setting up greedyProjectionMeshing Sphere objects." << std::endl;

        _in_SpherePointNormalCloud = _Sphere.pointCloudNormal();
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_SpherePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
        *_in_SpherePointNormalCloudPtr = _in_SpherePointNormalCloud;
        _out_FixtureSpherePolygonMesh = volcart::meshing::greedyProjectionMeshing(_in_SpherePointNormalCloudPtr, 100, 2.0, 2.5);
        pcl::io::loadOBJFile("SphereGreedyProjectionMeshing.obj", _in_SavedSpherePolygonMesh);
        pcl::fromPCLPointCloud2(_out_FixtureSpherePolygonMesh.cloud, _out_FixtureSpherePointCloud);
        pcl::fromPCLPointCloud2(_in_SavedSpherePolygonMesh.cloud, _out_SavedSpherePointCloud);
    }

    ~SphereGreedyProjectionFixture(){ std::cerr << "Cleaning up greedyProjectionMeshing Sphere objects" << std::endl; }

    volcart::shapes::Sphere _Sphere;
    pcl::PointCloud<pcl::PointNormal> _in_SpherePointNormalCloud;
    pcl::PolygonMesh _out_FixtureSpherePolygonMesh, _in_SavedSpherePolygonMesh;
    pcl::PointCloud<pcl::PointNormal> _out_FixtureSpherePointCloud, _out_SavedSpherePointCloud;

};

struct ConeGreedyProjectionFixture {

    ConeGreedyProjectionFixture() {

        std::cerr << "Setting up greedyProjectionMeshing Cone objects." << std::endl;

        _in_ConePointNormalCloud = _Cone.pointCloudNormal();
        pcl::PointCloud<pcl::PointNormal>::Ptr _in_ConePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
        *_in_ConePointNormalCloudPtr = _in_ConePointNormalCloud;
        _out_FixtureConePolygonMesh = volcart::meshing::greedyProjectionMeshing(_in_ConePointNormalCloudPtr, 100, 2.0, 2.5);
        pcl::io::loadOBJFile("ConeGreedyProjectionMeshing.obj", _in_SavedConePolygonMesh);
        pcl::fromPCLPointCloud2(_out_FixtureConePolygonMesh.cloud, _out_FixtureConePointCloud);
        pcl::fromPCLPointCloud2(_in_SavedConePolygonMesh.cloud, _out_SavedConePointCloud);

    }

    ~ConeGreedyProjectionFixture(){ std::cerr << "Cleaning up greedyProjectionMeshing Cone objects" << std::endl; }

    volcart::shapes::Cone _Cone;
    pcl::PointCloud<pcl::PointNormal> _in_ConePointNormalCloud;
    pcl::PolygonMesh _out_FixtureConePolygonMesh, _in_SavedConePolygonMesh;
    pcl::PointCloud<pcl::PointNormal> _out_FixtureConePointCloud, _out_SavedConePointCloud;

};

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixturePlaneGreedyProjections, PlaneGreedyProjectionFixture)
{
    // Check number of points and cells in each of the PolygonMesh objects for equality
    BOOST_CHECK_EQUAL (_out_FixturePlanePointCloud.points.size(), _out_SavedPlanePointCloud.points.size());
    BOOST_CHECK_EQUAL (_out_FixturePlanePolygonMesh.polygons.size(), _in_SavedPlanePolygonMesh.polygons.size());

    // Points
    for (int p = 0; p < _out_FixturePlanePointCloud.points.size(); p++) {
        for (int d = 0; d < 3; d++ ) {

            BOOST_CHECK_EQUAL (_out_FixturePlanePointCloud.points[p].data[d],
                                                                          _out_SavedPlanePointCloud.points[p].data[d]);
        }
    }

    // Cells
    for (int c_id = 0; c_id < _out_FixturePlanePolygonMesh.polygons.size(); c_id++) {

        for (int pnt = 0; pnt < _out_FixturePlanePolygonMesh.polygons[c_id].vertices.size(); pnt++) {

            //TODO: this is failing in debian_test_it job
            //TODO: fix
            BOOST_CHECK_EQUAL (_out_FixturePlanePolygonMesh.polygons[c_id].vertices[pnt],
                                                               _in_SavedPlanePolygonMesh.polygons[c_id].vertices[pnt]);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixtureCubeGreedyProjections, CubeGreedyProjectionFixture)
{
    // Check number of points and cells in each of the PolygonMesh objects for equality
    BOOST_CHECK_EQUAL (_out_FixtureCubePointCloud.points.size(), _out_SavedCubePointCloud.points.size());
    BOOST_CHECK_EQUAL (_out_FixtureCubePolygonMesh.polygons.size(), _in_SavedCubePolygonMesh.polygons.size());

    // Points
    for (int p = 0; p < _out_FixtureCubePointCloud.points.size(); p++) {
        for (int d = 0; d < 3; d++ ) {

            BOOST_CHECK_EQUAL (_out_FixtureCubePointCloud.points[p].data[d],
                                                                         _out_SavedCubePointCloud.points[p].data[d]);
        }
    }

    // Cells
    for (int c_id = 0; c_id < _out_FixtureCubePolygonMesh.polygons.size(); c_id++) {
        for (int pnt = 0; pnt < _out_FixtureCubePolygonMesh.polygons[c_id].vertices.size(); pnt++) {

            BOOST_CHECK_EQUAL (_out_FixtureCubePolygonMesh.polygons[c_id].vertices[pnt],
                                                               _in_SavedCubePolygonMesh.polygons[c_id].vertices[pnt]);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixtureArchGreedyProjections, ArchGreedyProjectionFixture)
{
    // Check number of points and cells in each of the PolygonMesh objects for equality
    BOOST_CHECK_EQUAL (_out_FixtureArchPointCloud.points.size(), _out_SavedArchPointCloud.points.size());
    BOOST_CHECK_EQUAL (_out_FixtureArchPolygonMesh.polygons.size(), _in_SavedArchPolygonMesh.polygons.size());

    // Points
    for (int p = 0; p < _out_FixtureArchPointCloud.points.size(); p++) {
        for (int d = 0; d < 3; d++ ) {

            BOOST_CHECK_CLOSE_FRACTION (_out_FixtureArchPointCloud.points[p].data[d],
                                                                  _out_SavedArchPointCloud.points[p].data[d], 0.0001);
        }
    }

    // Cells
    for (int c_id = 0; c_id < _out_FixtureArchPolygonMesh.polygons.size(); c_id++) {
        for (int pnt = 0; pnt < _out_FixtureArchPolygonMesh.polygons[c_id].vertices.size(); pnt++) {

            BOOST_CHECK_EQUAL (_out_FixtureArchPolygonMesh.polygons[c_id].vertices[pnt],
                                                               _in_SavedArchPolygonMesh.polygons[c_id].vertices[pnt]);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixtureSphereGreedyProjections, SphereGreedyProjectionFixture)
{
    // Check number of points and cells in each of the PolygonMesh objects for equality
    BOOST_CHECK_EQUAL (_out_FixtureSpherePointCloud.points.size(), _out_SavedSpherePointCloud.points.size());
    BOOST_CHECK_EQUAL (_out_FixtureSpherePolygonMesh.polygons.size(), _in_SavedSpherePolygonMesh.polygons.size());

    // Points
    for (int p = 0; p < _out_FixtureSpherePointCloud.points.size(); p++) {
        for (int d = 0; d < 3; d++ ) {

            BOOST_CHECK_CLOSE_FRACTION (_out_FixtureSpherePointCloud.points[p].data[d],
                                                                 _out_SavedSpherePointCloud.points[p].data[d], 0.0001);
        }
    }

    // Cells
    for (int c_id = 0; c_id < _out_FixtureSpherePolygonMesh.polygons.size(); c_id++) {
        for (int pnt = 0; pnt < _out_FixtureSpherePolygonMesh.polygons[c_id].vertices.size(); pnt++) {

            BOOST_CHECK_EQUAL (_out_FixtureSpherePolygonMesh.polygons[c_id].vertices[pnt],
                                                              _in_SavedSpherePolygonMesh.polygons[c_id].vertices[pnt]);
        }
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixtureConeGreedyProjections, ConeGreedyProjectionFixture)
{
    // Check number of points and cells in each of the PolygonMesh objects for equality
    BOOST_CHECK_EQUAL (_out_FixtureConePointCloud.points.size(), _out_SavedConePointCloud.points.size());
    BOOST_CHECK_EQUAL (_out_FixtureConePolygonMesh.polygons.size(), _in_SavedConePolygonMesh.polygons.size());

    // Points
    for (int p = 0; p < _out_FixtureConePointCloud.points.size(); p++) {
        for (int d = 0; d < 3; d++ ) {

            BOOST_CHECK_CLOSE_FRACTION (_out_FixtureConePointCloud.points[p].data[d],
                                                                  _out_SavedConePointCloud.points[p].data[d], 0.0001);
        }
    }

    // Cells
    for (int c_id = 0; c_id < _out_FixtureConePolygonMesh.polygons.size(); c_id++) {
        for (int pnt = 0; pnt < _out_FixtureConePolygonMesh.polygons[c_id].vertices.size(); pnt++) {

            BOOST_CHECK_EQUAL (_out_FixtureConePolygonMesh.polygons[c_id].vertices[pnt],
                                                               _in_SavedConePolygonMesh.polygons[c_id].vertices[pnt]);
        }
    }
}

