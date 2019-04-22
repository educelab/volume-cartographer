#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"

using namespace volcart;

/*
 *
 *    FIXTURES
 *
 */

class CreatePlaneABFUVFixture : public ::testing::Test
{
public:
    CreatePlaneABFUVFixture()
    {
        // Get ITK Mesh
        _in_Mesh = _Plane.itkMesh();

        // Create uvMap from mesh
        volcart::texturing::AngleBasedFlattening abf(_in_Mesh);
        abf.compute();
        _out_Mesh = abf.getMesh();

        // Load pre-generated output from file
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "abf_Plane.obj", _SavedPoints, _SavedCells);
    }

    // declare Plane mesh and width and height
    volcart::shapes::Plane _Plane;
    ITKMesh::Pointer _in_Mesh;
    ITKMesh::Pointer _out_Mesh;

    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class CreatePlaneABFLSCMOnlyUVFixture : public ::testing::Test
{
public:
    CreatePlaneABFLSCMOnlyUVFixture()
    {
        // Get ITK Mesh
        _in_Mesh = _Plane.itkMesh();

        // Create uvMap from mesh
        volcart::texturing::AngleBasedFlattening abf(_in_Mesh);
        abf.setUseABF(false);
        abf.compute();
        _out_Mesh = abf.getMesh();

        // Load pre-generated output from file
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "abf_Plane_LSCMOnly.obj", _SavedPoints, _SavedCells);
    }

    // declare Plane mesh and width and height
    volcart::shapes::Plane _Plane;
    ITKMesh::Pointer _in_Mesh;
    ITKMesh::Pointer _out_Mesh;

    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class CreateArchABFUVFixture : public ::testing::Test
{
public:
    CreateArchABFUVFixture()
    {
        // get ITK Mesh
        _in_Mesh = _Arch.itkMesh();

        // Create uvMap from mesh
        volcart::texturing::AngleBasedFlattening abf(_in_Mesh);
        abf.compute();
        _out_Mesh = abf.getMesh();

        // Load pre-generated output from file
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "abf_Arch.obj", _SavedPoints, _SavedCells);
    }

    // declare Arch mesh
    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_Mesh;
    ITKMesh::Pointer _out_Mesh;

    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class CreateArchABFLSCMOnlyUVFixture : public ::testing::Test
{
public:
    CreateArchABFLSCMOnlyUVFixture()
    {
        // get ITK Mesh
        _in_Mesh = _Arch.itkMesh();

        // Create uvMap from mesh
        volcart::texturing::AngleBasedFlattening abf(_in_Mesh);
        abf.setUseABF(false);
        abf.compute();
        _out_Mesh = abf.getMesh();

        // Load pre-generated output from file
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "abf_Arch_LSCMOnly.obj", _SavedPoints, _SavedCells);
    }

    // declare Arch mesh
    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_Mesh;
    ITKMesh::Pointer _out_Mesh;

    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

/*
 *
 *    TEST CASES
 *
 */

TEST_F(CreatePlaneABFUVFixture, PlaneABFUVTest)
{

    // check size of uvMap and number of points in mesh
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _in_Mesh->GetNumberOfPoints());
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // check uvmap against original mesh input pointIDs
    for (size_t point = 0; point < _SavedPoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[0], _SavedPoints[point].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[1], _SavedPoints[point].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[2], _SavedPoints[point].z);
    }
}

TEST_F(CreatePlaneABFLSCMOnlyUVFixture, PlaneABFLSCMOnlyUVTest)
{

    // check size of uvMap and number of points in mesh
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _in_Mesh->GetNumberOfPoints());
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // check uvmap against original mesh input pointIDs
    for (size_t point = 0; point < _SavedPoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[0], _SavedPoints[point].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[1], _SavedPoints[point].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[2], _SavedPoints[point].z);
    }
}

TEST_F(CreateArchABFUVFixture, ArchABFUVTest)
{

    // check size of uvMap and number of points in mesh
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _in_Mesh->GetNumberOfPoints());
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // check uvmap against original mesh input pointIDs
    for (size_t point = 0; point < _SavedPoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[0], _SavedPoints[point].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[1], _SavedPoints[point].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[2], _SavedPoints[point].z);
    }
}

TEST_F(CreateArchABFLSCMOnlyUVFixture, ArchABFLSCMOnlyUVTest)
{

    // check size of uvMap and number of points in mesh
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _in_Mesh->GetNumberOfPoints());
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // check uvmap against original mesh input pointIDs
    for (size_t point = 0; point < _SavedPoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[0], _SavedPoints[point].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[1], _SavedPoints[point].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[2], _SavedPoints[point].z);
    }
}