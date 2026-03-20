#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

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
        abf.setUseHLSCM(false);
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
        abf.setUseHLSCM(false);
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
        abf.setUseHLSCM(false);
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
        abf.setUseHLSCM(false);
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
    for (std::size_t point = 0; point < _SavedPoints.size(); ++point) {

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
    for (std::size_t point = 0; point < _SavedPoints.size(); ++point) {

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
    for (std::size_t point = 0; point < _SavedPoints.size(); ++point) {

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
    for (std::size_t point = 0; point < _SavedPoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[0], _SavedPoints[point].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[1], _SavedPoints[point].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(point)[2], _SavedPoints[point].z);
    }
}

/*
 *
 *    HLSCM VALIDITY TESTS
 *
 * These tests verify that HierarchicalLSCM produces valid UV output: correct
 * point count, correct face count, finite XZ coordinates, and Y=0 (flat
 * plane). They also verify that the parameterization is non-degenerate (not
 * all points collapsed to the same location).
 *
 */

static void CheckHLSCMValidity(
    const ITKMesh::Pointer& inMesh, const ITKMesh::Pointer& outMesh)
{
    ASSERT_EQ(outMesh->GetNumberOfPoints(), inMesh->GetNumberOfPoints());
    ASSERT_EQ(outMesh->GetNumberOfCells(), inMesh->GetNumberOfCells());

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();

    for (std::size_t i = 0; i < outMesh->GetNumberOfPoints(); ++i) {
        auto p = outMesh->GetPoint(i);
        EXPECT_TRUE(std::isfinite(p[0])) << "x not finite at point " << i;
        EXPECT_DOUBLE_EQ(p[1], 0.0) << "y != 0 at point " << i;
        EXPECT_TRUE(std::isfinite(p[2])) << "z not finite at point " << i;
        minX = std::min(minX, p[0]);
        maxX = std::max(maxX, p[0]);
        minZ = std::min(minZ, p[2]);
        maxZ = std::max(maxZ, p[2]);
    }

    // Verify non-degenerate: points span a non-zero area
    EXPECT_GT(maxX - minX, 0.0) << "All points have the same X coordinate";
    EXPECT_GT(maxZ - minZ, 0.0) << "All points have the same Z coordinate";
}

TEST(HLSCMTest, PlaneABFHLSCM)
{
    volcart::shapes::Plane plane;
    auto inMesh = plane.itkMesh();
    volcart::texturing::AngleBasedFlattening abf(inMesh);
    abf.setUseHLSCM(true);
    auto outMesh = abf.compute();
    CheckHLSCMValidity(inMesh, outMesh);
}

TEST(HLSCMTest, PlaneHLSCMOnly)
{
    volcart::shapes::Plane plane;
    auto inMesh = plane.itkMesh();
    volcart::texturing::AngleBasedFlattening abf(inMesh);
    abf.setUseABF(false);
    abf.setUseHLSCM(true);
    auto outMesh = abf.compute();
    CheckHLSCMValidity(inMesh, outMesh);
}

TEST(HLSCMTest, ArchABFHLSCM)
{
    volcart::shapes::Arch arch;
    auto inMesh = arch.itkMesh();
    volcart::texturing::AngleBasedFlattening abf(inMesh);
    abf.setUseHLSCM(true);
    auto outMesh = abf.compute();
    CheckHLSCMValidity(inMesh, outMesh);
}

TEST(HLSCMTest, ArchHLSCMOnly)
{
    volcart::shapes::Arch arch;
    auto inMesh = arch.itkMesh();
    volcart::texturing::AngleBasedFlattening abf(inMesh);
    abf.setUseABF(false);
    abf.setUseHLSCM(true);
    auto outMesh = abf.compute();
    CheckHLSCMValidity(inMesh, outMesh);
}
