//
// Created by Ryan Taber on 11/30/15.
//

#define BOOST_TEST_MODULE RayTrace

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "core/shapes/Arch.h"
#include "core/shapes/Cone.h"
#include "core/shapes/Cube.h"
#include "core/shapes/Plane.h"
#include "core/shapes/Sphere.h"
#include "core/vc_defines.h"
#include "meshing/RayTrace.h"
#include "testing/ParsingHelpers.h"
#include "testing/TestingUtils.h"

using namespace volcart;

/************************************************************************************
 *                                                                                  *
 *  RayTraceTest.cpp - tests the functionality of *
 *  v-c/meshing/RayTrace.cpp with the ultimate goal of the following: *
 *                                                                                  *
 *     Given the same curved input mesh, does a saved PLY file match a current *
 *     execution of RayTrace(). *
 *                                                                                  *
 *  This file is broken up into a test fixture RayTraceFix which initialize *
 *  the objects used in any subsequent fixture test cases. *
 *                                                                                  *
 *  Test Cases: *
 *  1. SavedPlaneRayTraceComparison (PlaneRayTraceFixture) *
 *  2. SavedCubeRayTraceComparison (CubeRayTraceFixture) *
 *  3. SavedArchRayTraceComparison (ArchRayTraceFixture) *
 *  4. SavedSphereRayTraceComparison (SphereRayTraceFixture) *
 *  5. SavedConeRayTraceComparison (ConeRayTraceFixture) *
 *                                                                                  *
 * *
 *  Input: *
 *     No required inputs for the test cases. Any test objects are created *
 *     internally by the various Fixtures() or within the test cases themselves.
 * *
 *                                                                                  *
 *  Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general  *
 *     number of testing errors is output to console. *
 *                                                                                  *
 *  Miscellaneous: *
 *     See the /testing/meshing wiki for more information on this test *
 * **********************************************************************************/

struct PlaneRayTraceFixture {

    PlaneRayTraceFixture()
    {

        // generate the curved mesh
        _in_PlaneMesh = _Plane.itkMesh();

        // call RayTrace() and assign results
        _PlaneRayTraceResults = volcart::meshing::RayTrace(
            _in_PlaneMesh, _TraceDir, _Width, _Height, _UVMap);

        //
        // Write ray trace results to file
        //

        _NumberOfPointsInMesh = _PlaneRayTraceResults.size();

        _SavedPlaneMeshFile.open("TestPlaneRayTraceData.ply");

        std::cout << "Writing RayTrace results to file..." << std::endl;

        // write header
        _SavedPlaneMeshFile
            << "ply" << std::endl
            << "format ascii 1.0" << std::endl
            << "comment Created by particle simulation "
               "https://github.com/viscenter/registration-toolkit"
            << std::endl
            << "element vertex " << _NumberOfPointsInMesh << std::endl
            << "property float x" << std::endl
            << "property float y" << std::endl
            << "property float z" << std::endl
            << "property float nx" << std::endl
            << "property float ny" << std::endl
            << "property float nz" << std::endl
            << "element face 0" << std::endl
            << "property list uchar int vertex_indices" << std::endl
            << "end_header" << std::endl;

        // write vertex information
        for (int i = 0; i < _NumberOfPointsInMesh; i++) {

            // x y z nx ny nz
            _SavedPlaneMeshFile << _PlaneRayTraceResults[i](0) << " "
                                << _PlaneRayTraceResults[i](1) << " "
                                << _PlaneRayTraceResults[i](2) << " "
                                << _PlaneRayTraceResults[i](3) << " "
                                << _PlaneRayTraceResults[i](4) << " ";

            // Hack to get rid of "-0" values that appeared in the
            // saved file the first time this was run

            if (_PlaneRayTraceResults[i](5) == -0) {
                _SavedPlaneMeshFile << "0 " << std::endl;
            } else {
                _SavedPlaneMeshFile << _PlaneRayTraceResults[i](5) << " "
                                    << std::endl;
            }
        }

        _SavedPlaneMeshFile.close();

        // Read in data from .ply files

        volcart::testing::ParsingHelpers::parsePlyFile(
            "SavedPlaneRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::parsePlyFile(
            "TestPlaneRayTraceData.ply", _CurrentPoints, _CurrentCells);

        std::cerr << "\nsetting up PlaneRayTraceTest objects" << std::endl;
    }

    ~PlaneRayTraceFixture()
    {
        std::cerr << "\ncleaning up PlaneRayTraceTest objects" << std::endl;
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _PlaneRayTraceResults;
    ITKMesh::Pointer _in_PlaneMesh;
    volcart::shapes::Plane _Plane;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;  // default direction is anything != 1
    int _Width, _Height;

    // Variables for writing to file
    unsigned long _NumberOfPointsInMesh;
    std::ofstream _SavedPlaneMeshFile;

    // Vectors that will hold points and cells
    std::vector<Vertex> _SavedPoints, _CurrentPoints;
    std::vector<Cell> _SavedCells, _CurrentCells;
};

struct CubeRayTraceFixture {

    CubeRayTraceFixture()
    {

        // generate the curved mesh
        _in_CubeMesh = _Cube.itkMesh();

        // call RayTrace() and assign results
        _CubeRayTraceResults = volcart::meshing::RayTrace(
            _in_CubeMesh, _TraceDir, _Width, _Height, _UVMap);

        //
        // Write ray trace results to file
        //

        _NumberOfPointsInMesh = _CubeRayTraceResults.size();

        _SavedCubeMeshFile.open("TestCubeRayTraceData.ply");

        std::cout << "Writing RayTrace results to file..." << std::endl;

        // write header
        _SavedCubeMeshFile << "ply" << std::endl
                           << "format ascii 1.0" << std::endl
                           << "comment Created by particle simulation "
                              "https://github.com/viscenter/"
                              "registration-toolkit"
                           << std::endl
                           << "element vertex " << _NumberOfPointsInMesh
                           << std::endl
                           << "property float x" << std::endl
                           << "property float y" << std::endl
                           << "property float z" << std::endl
                           << "property float nx" << std::endl
                           << "property float ny" << std::endl
                           << "property float nz" << std::endl
                           << "element face 0" << std::endl
                           << "property list uchar int vertex_indices"
                           << std::endl
                           << "end_header" << std::endl;

        // write vertex information
        for (int i = 0; i < _NumberOfPointsInMesh; i++) {

            // x y z nx ny nz
            _SavedCubeMeshFile << _CubeRayTraceResults[i](0) << " "
                               << _CubeRayTraceResults[i](1) << " "
                               << _CubeRayTraceResults[i](2) << " "
                               << _CubeRayTraceResults[i](3) << " "
                               << _CubeRayTraceResults[i](4) << " ";

            // Hack to get rid of "-0" values that appeared in the
            // saved file the first time this was run

            if (_CubeRayTraceResults[i](5) == -0) {
                _SavedCubeMeshFile << "0 " << std::endl;
            } else {
                _SavedCubeMeshFile << _CubeRayTraceResults[i](5) << " "
                                   << std::endl;
            }
        }

        _SavedCubeMeshFile.close();

        // Read in data from .ply files
        volcart::testing::ParsingHelpers::parsePlyFile(
            "SavedCubeRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::parsePlyFile(
            "TestCubeRayTraceData.ply", _CurrentPoints, _CurrentCells);

        std::cerr << "\nsetting up CubeRayTraceTest objects" << std::endl;
    }

    ~CubeRayTraceFixture()
    {
        std::cerr << "\ncleaning up CubeRayTraceTest objects" << std::endl;
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _CubeRayTraceResults;
    ITKMesh::Pointer _in_CubeMesh;
    volcart::shapes::Cube _Cube;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;
    int _Width, _Height;

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedCubeMeshFile;

    // Vectors that will hold points and cells
    std::vector<Vertex> _SavedPoints, _CurrentPoints;
    std::vector<Cell> _SavedCells, _CurrentCells;
};

struct ArchRayTraceFixture {

    ArchRayTraceFixture()
    {

        // generate the curved mesh
        _in_ArchMesh = _Arch.itkMesh();

        // call RayTrace() and assign results
        _ArchRayTraceResults = volcart::meshing::RayTrace(
            _in_ArchMesh, _TraceDir, _Width, _Height, _UVMap);

        //
        // Write ray trace results to file
        //

        _NumberOfPointsInMesh = _ArchRayTraceResults.size();

        _SavedArchMeshFile.open("TestArchRayTraceData.ply");

        std::cout << "Writing RayTrace results to file..." << std::endl;

        // write header
        _SavedArchMeshFile << "ply" << std::endl
                           << "format ascii 1.0" << std::endl
                           << "comment Created by particle simulation "
                              "https://github.com/viscenter/"
                              "registration-toolkit"
                           << std::endl
                           << "element vertex " << _NumberOfPointsInMesh
                           << std::endl
                           << "property float x" << std::endl
                           << "property float y" << std::endl
                           << "property float z" << std::endl
                           << "property float nx" << std::endl
                           << "property float ny" << std::endl
                           << "property float nz" << std::endl
                           << "element face 0" << std::endl
                           << "property list uchar int vertex_indices"
                           << std::endl
                           << "end_header" << std::endl;

        // write vertex information
        for (int i = 0; i < _NumberOfPointsInMesh; i++) {

            // x y z nx ny nz
            _SavedArchMeshFile << _ArchRayTraceResults[i](0) << " "
                               << _ArchRayTraceResults[i](1) << " "
                               << _ArchRayTraceResults[i](2) << " "
                               << _ArchRayTraceResults[i](3) << " "
                               << _ArchRayTraceResults[i](4) << " ";

            // Hack to get rid of "-0" values that appeared in the
            // saved file the first time this was run

            if (_ArchRayTraceResults[i](5) == -0) {
                _SavedArchMeshFile << "0 " << std::endl;
            } else {
                _SavedArchMeshFile << _ArchRayTraceResults[i](5) << " "
                                   << std::endl;
            }
        }

        _SavedArchMeshFile.close();

        // Read in data from .ply files
        volcart::testing::ParsingHelpers::parsePlyFile(
            "SavedArchRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::parsePlyFile(
            "TestArchRayTraceData.ply", _CurrentPoints, _CurrentCells);

        std::cerr << "\nsetting up ArchRayTraceTest objects" << std::endl;
    }

    ~ArchRayTraceFixture()
    {
        std::cerr << "\ncleaning up ArchRayTraceTest objects" << std::endl;
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _ArchRayTraceResults;
    ITKMesh::Pointer _in_ArchMesh;
    volcart::shapes::Arch _Arch;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;  // default direction is anything != 1
    int _Width, _Height;

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedArchMeshFile;

    // Vectors that will hold points and cells
    std::vector<Vertex> _SavedPoints, _CurrentPoints;
    std::vector<Cell> _SavedCells, _CurrentCells;
};

struct SphereRayTraceFixture {

    SphereRayTraceFixture()
    {

        // generate the curved mesh
        _in_SphereMesh = _Sphere.itkMesh();

        // call RayTrace() and assign results
        _SphereRayTraceResults = volcart::meshing::RayTrace(
            _in_SphereMesh, _TraceDir, _Width, _Height, _UVMap);

        //
        // Write ray trace results to file
        //

        _NumberOfPointsInMesh = _SphereRayTraceResults.size();

        _SavedSphereMeshFile.open("TestSphereRayTraceData.ply");

        std::cout << "Writing RayTrace results to file..." << std::endl;

        // write header
        _SavedSphereMeshFile
            << "ply" << std::endl
            << "format ascii 1.0" << std::endl
            << "comment Created by particle simulation "
               "https://github.com/viscenter/registration-toolkit"
            << std::endl
            << "element vertex " << _NumberOfPointsInMesh << std::endl
            << "property float x" << std::endl
            << "property float y" << std::endl
            << "property float z" << std::endl
            << "property float nx" << std::endl
            << "property float ny" << std::endl
            << "property float nz" << std::endl
            << "element face 0" << std::endl
            << "property list uchar int vertex_indices" << std::endl
            << "end_header" << std::endl;

        // write vertex information
        for (int i = 0; i < _NumberOfPointsInMesh; i++) {

            // x y z nx ny nz
            _SavedSphereMeshFile << _SphereRayTraceResults[i](0) << " "
                                 << _SphereRayTraceResults[i](1) << " "
                                 << _SphereRayTraceResults[i](2) << " "
                                 << _SphereRayTraceResults[i](3) << " "
                                 << _SphereRayTraceResults[i](4) << " ";

            // Hack to get rid of "-0" values that appeared in the
            // saved file the first time this was run

            if (_SphereRayTraceResults[i](5) == -0) {
                _SavedSphereMeshFile << "0 " << std::endl;
            } else {
                _SavedSphereMeshFile << _SphereRayTraceResults[i](5) << " "
                                     << std::endl;
            }
        }

        _SavedSphereMeshFile.close();

        // Read in data from .ply files

        volcart::testing::ParsingHelpers::parsePlyFile(
            "SavedSphereRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::parsePlyFile(
            "TestSphereRayTraceData.ply", _CurrentPoints, _CurrentCells);

        std::cerr << "\nsetting up SphereRayTraceTest objects" << std::endl;
    }

    ~SphereRayTraceFixture()
    {
        std::cerr << "\ncleaning up SphereRayTraceTest objects" << std::endl;
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _SphereRayTraceResults;
    ITKMesh::Pointer _in_SphereMesh;
    volcart::shapes::Sphere _Sphere;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;
    int _Width, _Height;

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedSphereMeshFile;

    // Vectors that will hold points and cells
    std::vector<Vertex> _SavedPoints, _CurrentPoints;
    std::vector<Cell> _SavedCells, _CurrentCells;
};

struct ConeRayTraceFixture {

    ConeRayTraceFixture()
    {

        // generate the curved mesh
        _in_ConeMesh = _Cone.itkMesh();

        // call RayTrace() and assign results
        _ConeRayTraceResults = volcart::meshing::RayTrace(
            _in_ConeMesh, _TraceDir, _Width, _Height, _UVMap);

        //
        // Write ray trace results to file
        //

        _NumberOfPointsInMesh = _ConeRayTraceResults.size();

        _SavedConeMeshFile.open("TestConeRayTraceData.ply");

        std::cout << "Writing RayTrace results to file..." << std::endl;

        // write header
        _SavedConeMeshFile << "ply" << std::endl
                           << "format ascii 1.0" << std::endl
                           << "comment Created by particle simulation "
                              "https://github.com/viscenter/"
                              "registration-toolkit"
                           << std::endl
                           << "element vertex " << _NumberOfPointsInMesh
                           << std::endl
                           << "property float x" << std::endl
                           << "property float y" << std::endl
                           << "property float z" << std::endl
                           << "property float nx" << std::endl
                           << "property float ny" << std::endl
                           << "property float nz" << std::endl
                           << "element face 0" << std::endl
                           << "property list uchar int vertex_indices"
                           << std::endl
                           << "end_header" << std::endl;

        // write vertex information
        for (int i = 0; i < _NumberOfPointsInMesh; i++) {

            // x y z nx ny nz
            _SavedConeMeshFile << _ConeRayTraceResults[i](0) << " "
                               << _ConeRayTraceResults[i](1) << " "
                               << _ConeRayTraceResults[i](2) << " "
                               << _ConeRayTraceResults[i](3) << " "
                               << _ConeRayTraceResults[i](4) << " ";

            // Hack to get rid of "-0" values that appeared in the
            // saved file the first time this was run

            if (_ConeRayTraceResults[i](5) == -0) {
                _SavedConeMeshFile << "0 " << std::endl;
            } else {
                _SavedConeMeshFile << _ConeRayTraceResults[i](5) << " "
                                   << std::endl;
            }
        }

        _SavedConeMeshFile.close();

        // Read in data from .ply files

        volcart::testing::ParsingHelpers::parsePlyFile(
            "SavedConeRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::parsePlyFile(
            "TestConeRayTraceData.ply", _CurrentPoints, _CurrentCells);

        std::cerr << "\nsetting up ConeRayTraceTest objects" << std::endl;
    }

    ~ConeRayTraceFixture()
    {
        std::cerr << "\ncleaning up ConeRayTraceTest objects" << std::endl;
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _ConeRayTraceResults;
    ITKMesh::Pointer _in_ConeMesh;
    volcart::shapes::Cone _Cone;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;
    int _Width, _Height;

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedConeMeshFile;

    // Vectors that will hold points and cells
    std::vector<Vertex> _SavedPoints, _CurrentPoints;
    std::vector<Cell> _SavedCells, _CurrentCells;
};

BOOST_FIXTURE_TEST_CASE(SavedPlaneRayTraceComparison, PlaneRayTraceFixture)
{

    // Compare the saved and fixture-created raytrace() points
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (int point = 0; point < _SavedPoints.size(); point++) {

        volcart::testing::SmallOrClose(
            _SavedPoints[point].x, _CurrentPoints[point].x);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].y, _CurrentPoints[point].y);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].z, _CurrentPoints[point].z);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nx, _CurrentPoints[point].nx);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].ny, _CurrentPoints[point].ny);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

BOOST_FIXTURE_TEST_CASE(SavedCubeRayTraceComparison, CubeRayTraceFixture)
{

    // Compare the saved and fixture-created raytrace() points
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (int point = 0; point < _SavedPoints.size(); point++) {

        volcart::testing::SmallOrClose(
            _SavedPoints[point].x, _CurrentPoints[point].x);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].y, _CurrentPoints[point].y);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].z, _CurrentPoints[point].z);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nx, _CurrentPoints[point].nx);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].ny, _CurrentPoints[point].ny);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

BOOST_FIXTURE_TEST_CASE(SavedArchRayTraceComparison, ArchRayTraceFixture)
{

    // Compare the saved and fixture-created raytrace() points
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _CurrentPoints.size());

    // check points
    for (int point = 0; point < _SavedPoints.size(); point++) {

        volcart::testing::SmallOrClose(
            _SavedPoints[point].x, _CurrentPoints[point].x);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].y, _CurrentPoints[point].y);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].z, _CurrentPoints[point].z);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nx, _CurrentPoints[point].nx);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].ny, _CurrentPoints[point].ny);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

BOOST_FIXTURE_TEST_CASE(SavedSphereRayTraceComparison, SphereRayTraceFixture)
{

    // Compare the saved and fixture-created raytrace() points
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (int point = 0; point < _SavedPoints.size(); point++) {

        volcart::testing::SmallOrClose(
            _SavedPoints[point].x, _CurrentPoints[point].x);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].y, _CurrentPoints[point].y);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].z, _CurrentPoints[point].z);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nx, _CurrentPoints[point].nx);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].ny, _CurrentPoints[point].ny);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

BOOST_FIXTURE_TEST_CASE(SavedConeRayTraceComparison, ConeRayTraceFixture)
{

    // Compare the saved and fixture-created raytrace() points
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (int point = 0; point < _SavedPoints.size(); point++) {

        volcart::testing::SmallOrClose(
            _SavedPoints[point].x, _CurrentPoints[point].x);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].y, _CurrentPoints[point].y);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].z, _CurrentPoints[point].z);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nx, _CurrentPoints[point].nx);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].ny, _CurrentPoints[point].ny);
        volcart::testing::SmallOrClose(
            _SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}
