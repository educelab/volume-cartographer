//
// Created by Ryan Taber on 3/3/16.
//

#define BOOST_TEST_MODULE simpleUV

#include <boost/test/unit_test.hpp>
#include "common/shapes/Arch.h"
#include "common/shapes/Plane.h"
#include "common/vc_defines.h"
#include "testing/testingUtils.h"
#include "texturing/simpleUV.h"

using namespace volcart;

/***************************************************************************************
 *                                                                                     *
 *  simpleUVTest.cpp - tests the functionality of /v-c/texturing/simpleUV.cpp *
 *  The ultimate goal of this file is the following: *
 *                                                                                     *
 *    Confirm that a uvMap created from mesh encodes the pointIDs of the *
 *    mesh in <u,v> coords. *
 *                                                                                     *
 *  This file is broken up into a test fixture which initializes the objects *
 *  used in each of the test cases. *
 *                                                                                     *
 *   1. PlaneSimpleUVTest *
 *   2. ArchSimpleUVTest *
 *                                                                                     *
 * Input: *
 *     No required inputs for this sample test. All objects needed for the test
 * cases  *
 *     are constructed and destroyed by the fixtures. Note, we're only testing
 * the     *
 *     non-enclosed shapes here. Sphere, Cone and Cube have been omitted. *
 *                                                                                     *
 * Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general     *
 *     number of testing errors is output. *
 *                                                                                     *
 * *************************************************************************************/

/*
 *
 *    FIXTURES
 *
 */

struct CreatePlaneSimpleUVFixture {

    CreatePlaneSimpleUVFixture()
    {

        std::cerr << "Creating Plane simple uv map..." << std::endl;

        // create ordered point cloud from shape and assign width and height
        // params: false noise and true ordered
        _in_PlanePointCloud = _Plane.orderedPoints(true);
        _width = _in_PlanePointCloud.width();
        _height = _in_PlanePointCloud.height();

        _in_PlaneITKMesh = _Plane.itkMesh();

        // create uvMap from mesh, width and height
        _out_PlaneUVMap =
            volcart::texturing::simpleUV(_in_PlaneITKMesh, _width, _height);
    }

    ~CreatePlaneSimpleUVFixture()
    {
        std::cerr << "Destroying Plane simple uv map..." << std::endl;
    }

    // declare Plane mesh and width and height
    volcart::shapes::Plane _Plane;
    volcart::ITKMesh::Pointer _in_PlaneITKMesh;
    volcart::OrderedPointSet<volcart::Point3d> _in_PlanePointCloud;
    int _width, _height;

    // declare uvMap to hold output from simpleUV call
    volcart::UVMap _out_PlaneUVMap;
};

struct CreateArchSimpleUVFixture {

    CreateArchSimpleUVFixture()
    {

        std::cerr << "Creating Arch simple uv map..." << std::endl;

        // create ordered point cloud from shape and assign width and height
        // params: false noise and true ordered
        _in_ArchPointCloud = _Arch.orderedPoints(true);
        _width = _in_ArchPointCloud.width();
        _height = _in_ArchPointCloud.height();

        _in_ArchITKMesh = _Arch.itkMesh();

        // create uvMap from mesh, width and height
        _out_ArchUVMap =
            volcart::texturing::simpleUV(_in_ArchITKMesh, _width, _height);
    }

    ~CreateArchSimpleUVFixture()
    {
        std::cerr << "Destroying Arch simple uv map..." << std::endl;
    }

    // declare Arch mesh
    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_ArchITKMesh;
    volcart::UVMap _out_ArchUVMap;

    volcart::OrderedPointSet<volcart::Point3d> _in_ArchPointCloud;
    int _width, _height;
};

/*
 *
 *    TEST CASES
 *
 */

BOOST_FIXTURE_TEST_CASE(PlaneSimpleUVTest, CreatePlaneSimpleUVFixture)
{

    // check size of uvMap and number of points in mesh
    BOOST_CHECK_EQUAL(
        _out_PlaneUVMap.size(), _in_PlaneITKMesh->GetNumberOfPoints());

    double u_Fixture, v_Fixture, ArrayX, ArrayY, u_Generated, v_Generated;

    // check uvmap against original mesh input pointIDs
    for (auto pnt = _in_PlaneITKMesh->GetPoints()->Begin();
         pnt != _in_PlaneITKMesh->GetPoints()->End(); ++pnt) {

        // calculate what the UV should be
        ArrayX = pnt.Index() % _width;
        ArrayY = (pnt.Index() - ArrayX) / _width;

        // u and v are [ArrayX, ArrayY] / [max_x, max_y]
        // max_x and max_y are the maximum index numbers
        // Since we're zero-indexed, they are width and height - 1
        u_Generated = ArrayX / (_width - 1);
        v_Generated = ArrayY / (_height - 1);

        // get u,v for point in map
        u_Fixture = _out_PlaneUVMap.get(pnt.Index())[0];
        v_Fixture = _out_PlaneUVMap.get(pnt.Index())[1];

        // check to see the values of scaled u,v values correspond to the proper
        // coords for current mesh pointID
        volcart::testing::SmallOrClose(u_Generated, u_Fixture);
        volcart::testing::SmallOrClose(v_Generated, v_Fixture);
    }
}

BOOST_FIXTURE_TEST_CASE(ArchSimpleUVTest, CreateArchSimpleUVFixture)
{

    // check size of uvMap and number of points in mesh
    BOOST_CHECK_EQUAL(
        _out_ArchUVMap.size(), _in_ArchITKMesh->GetNumberOfPoints());

    double u_Fixture, v_Fixture, ArrayX, ArrayY, u_Generated, v_Generated;

    // check uvmap against original mesh input pointIDs
    for (auto pnt = _in_ArchITKMesh->GetPoints()->Begin();
         pnt != _in_ArchITKMesh->GetPoints()->End(); ++pnt) {

        // calculate what the UV should be
        ArrayX = pnt.Index() % _width;
        ArrayY = (pnt.Index() - ArrayX) / _width;

        // u and v are [ArrayX, ArrayY] / [max_x, max_y]
        // max_x and max_y are the maximum index numbers
        // Since we're zero-indexed, they are width and height - 1
        u_Generated = ArrayX / (_width - 1);
        v_Generated = ArrayY / (_height - 1);

        // get u,v for point in map
        u_Fixture = _out_ArchUVMap.get(pnt.Index())[0];
        v_Fixture = _out_ArchUVMap.get(pnt.Index())[1];

        // check to see the values of scaled u,v values correspond to the proper
        // coords for current mesh pointID
        volcart::testing::SmallOrClose(u_Generated, u_Fixture);
        volcart::testing::SmallOrClose(v_Generated, v_Fixture);
    }
}
