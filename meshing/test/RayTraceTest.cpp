#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Cone.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Sphere.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/meshing/RayTrace.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;

class PlaneRayTraceFixture : public ::testing::Test
{
public:
    PlaneRayTraceFixture()
    {

        // generate the curved mesh
        _in_PlaneMesh = _Plane.itkMesh();

        // call RayTrace() and assign results
        _PlaneRayTraceResults = volcart::meshing::RayTrace(
            _in_PlaneMesh, _TraceDir, _Width, _Height, _UVMap);

        // Write ray trace results to file

        _NumberOfPointsInMesh = _PlaneRayTraceResults.size();

        _SavedPlaneMeshFile.open("TestPlaneRayTraceData.ply");

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
        for (uint32_t i = 0; i < _NumberOfPointsInMesh; i++) {

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

        volcart::testing::ParsingHelpers::ParsePLYFile(
            "SavedPlaneRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "TestPlaneRayTraceData.ply", _CurrentPoints, _CurrentCells);
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _PlaneRayTraceResults;
    ITKMesh::Pointer _in_PlaneMesh;
    volcart::shapes::Plane _Plane;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;  // default direction is anything != 1
    int _Width{0}, _Height{0};

    // Variables for writing to file
    uint64_t _NumberOfPointsInMesh;
    std::ofstream _SavedPlaneMeshFile;

    // Vectors that will hold points and cells
    std::vector<SimpleMesh::Vertex> _SavedPoints, _CurrentPoints;
    std::vector<SimpleMesh::Cell> _SavedCells, _CurrentCells;
};

class CubeRayTraceFixture : public ::testing::Test
{
public:
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
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "SavedCubeRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "TestCubeRayTraceData.ply", _CurrentPoints, _CurrentCells);
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _CubeRayTraceResults;
    ITKMesh::Pointer _in_CubeMesh;
    volcart::shapes::Cube _Cube;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;
    int _Width{0}, _Height{0};

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedCubeMeshFile;

    // Vectors that will hold points and cells
    std::vector<SimpleMesh::Vertex> _SavedPoints, _CurrentPoints;
    std::vector<SimpleMesh::Cell> _SavedCells, _CurrentCells;
};

class ArchRayTraceFixture : public ::testing::Test
{
public:
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
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "SavedArchRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "TestArchRayTraceData.ply", _CurrentPoints, _CurrentCells);
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _ArchRayTraceResults;
    ITKMesh::Pointer _in_ArchMesh;
    volcart::shapes::Arch _Arch;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;  // default direction is anything != 1
    int _Width{0}, _Height{0};

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedArchMeshFile;

    // Vectors that will hold points and cells
    std::vector<SimpleMesh::Vertex> _SavedPoints, _CurrentPoints;
    std::vector<SimpleMesh::Cell> _SavedCells, _CurrentCells;
};

class SphereRayTraceFixture : public ::testing::Test
{
public:
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

        volcart::testing::ParsingHelpers::ParsePLYFile(
            "SavedSphereRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "TestSphereRayTraceData.ply", _CurrentPoints, _CurrentCells);
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _SphereRayTraceResults;
    ITKMesh::Pointer _in_SphereMesh;
    volcart::shapes::Sphere _Sphere;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;
    int _Width{0}, _Height{0};

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedSphereMeshFile;

    // Vectors that will hold points and cells
    std::vector<SimpleMesh::Vertex> _SavedPoints, _CurrentPoints;
    std::vector<SimpleMesh::Cell> _SavedCells, _CurrentCells;
};

class ConeRayTraceFixture : public ::testing::Test
{
public:
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

        volcart::testing::ParsingHelpers::ParsePLYFile(
            "SavedConeRayTraceData.ply", _SavedPoints, _SavedCells);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "TestConeRayTraceData.ply", _CurrentPoints, _CurrentCells);
    }

    // Variables needed for call to RayTrace()
    std::vector<cv::Vec6f> _ConeRayTraceResults;
    ITKMesh::Pointer _in_ConeMesh;
    volcart::shapes::Cone _Cone;
    std::map<int, cv::Vec2d> _UVMap;
    int _TraceDir = 0;
    int _Width{0}, _Height{0};

    // Variables for writing to file
    int _NumberOfPointsInMesh;
    std::ofstream _SavedConeMeshFile;

    // Vectors that will hold points and cells
    std::vector<SimpleMesh::Vertex> _SavedPoints, _CurrentPoints;
    std::vector<SimpleMesh::Cell> _SavedCells, _CurrentCells;
};

TEST_F(PlaneRayTraceFixture, SavedPlaneRayTraceComparison)
{

    // Compare the saved and fixture-created raytrace() points
    EXPECT_EQ(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (size_t point = 0; point < _SavedPoints.size(); point++) {

        EXPECT_DOUBLE_EQ(_SavedPoints[point].x, _CurrentPoints[point].x);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].y, _CurrentPoints[point].y);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].z, _CurrentPoints[point].z);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nx, _CurrentPoints[point].nx);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].ny, _CurrentPoints[point].ny);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

TEST_F(CubeRayTraceFixture, SavedCubeRayTraceComparison)
{

    // Compare the saved and fixture-created raytrace() points
    EXPECT_EQ(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (size_t point = 0; point < _SavedPoints.size(); point++) {

        EXPECT_DOUBLE_EQ(_SavedPoints[point].x, _CurrentPoints[point].x);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].y, _CurrentPoints[point].y);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].z, _CurrentPoints[point].z);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nx, _CurrentPoints[point].nx);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].ny, _CurrentPoints[point].ny);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

TEST_F(ArchRayTraceFixture, SavedArchRayTraceComparison)
{

    // Compare the saved and fixture-created raytrace() points
    EXPECT_EQ(_SavedPoints.size(), _CurrentPoints.size());

    // check points
    for (size_t point = 0; point < _SavedPoints.size(); point++) {

        EXPECT_DOUBLE_EQ(_SavedPoints[point].x, _CurrentPoints[point].x);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].y, _CurrentPoints[point].y);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].z, _CurrentPoints[point].z);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nx, _CurrentPoints[point].nx);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].ny, _CurrentPoints[point].ny);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

TEST_F(SphereRayTraceFixture, SavedSphereRayTraceComparison)
{

    // Compare the saved and fixture-created raytrace() points
    EXPECT_EQ(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (size_t point = 0; point < _SavedPoints.size(); point++) {

        EXPECT_DOUBLE_EQ(_SavedPoints[point].x, _CurrentPoints[point].x);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].y, _CurrentPoints[point].y);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].z, _CurrentPoints[point].z);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nx, _CurrentPoints[point].nx);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].ny, _CurrentPoints[point].ny);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}

TEST_F(ConeRayTraceFixture, SavedConeRayTraceComparison)
{

    // Compare the saved and fixture-created raytrace() points
    EXPECT_EQ(_SavedPoints.size(), _CurrentPoints.size());

    // loop over points
    for (size_t point = 0; point < _SavedPoints.size(); point++) {

        EXPECT_DOUBLE_EQ(_SavedPoints[point].x, _CurrentPoints[point].x);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].y, _CurrentPoints[point].y);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].z, _CurrentPoints[point].z);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nx, _CurrentPoints[point].nx);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].ny, _CurrentPoints[point].ny);
        EXPECT_DOUBLE_EQ(_SavedPoints[point].nz, _CurrentPoints[point].nz);
    }
}