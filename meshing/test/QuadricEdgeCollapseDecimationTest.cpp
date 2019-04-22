#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Cone.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Sphere.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/meshing/CalculateNormals.hpp"
#include "vc/meshing/QuadricEdgeCollapseDecimation.hpp"
#include "vc/testing/ParsingHelpers.hpp"

using namespace volcart;

// Setting up the shapes
class QuadricPlaneFixture : public ::testing::Test
{
public:
    QuadricPlaneFixture()
    {
        _Plane = volcart::shapes::Plane(10, 10);
        _in_Mesh = _Plane.itkMesh();
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "QuadricEdgeCollapse_Plane.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Plane _Plane;
    ITKMesh::Pointer _in_Mesh, _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class QuadricArchFixture : public ::testing::Test
{
public:
    QuadricArchFixture()
    {
        _Arch = volcart::shapes::Arch(100, 100);
        _in_Mesh = _Arch.itkMesh();
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "QuadricEdgeCollapse_Arch.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_Mesh, _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class QuadricConeFixture : public ::testing::Test
{
public:
    QuadricConeFixture()
    {
        _Cone = volcart::shapes::Cone(1000, 1000);
        _in_Mesh = _Cone.itkMesh();
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "QuadricEdgeCollapse_Cone.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Cone _Cone;
    ITKMesh::Pointer _in_Mesh, _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class QuadricCubeFixture : public ::testing::Test
{
public:
    QuadricCubeFixture()
    {
        _Cube = volcart::shapes::Cube();
        _in_Mesh = _Cube.itkMesh();
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "QuadricEdgeCollapse_Cube.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Cube _Cube;
    ITKMesh::Pointer _in_Mesh, _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class QuadricSphereFixture : public ::testing::Test
{
public:
    QuadricSphereFixture()
    {
        _Sphere = volcart::shapes::Sphere(30, 3);
        _in_Mesh = _Sphere.itkMesh();
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "QuadricEdgeCollapse_Sphere.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Sphere _Sphere;
    ITKMesh::Pointer _in_Mesh, _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

// Tests by Shape
TEST_F(QuadricPlaneFixture, QuadricResampledPlaneTest)
{
    volcart::meshing::QuadricEdgeCollapseDecimation resample;
    resample.setMesh(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells() / 2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm(_out_Mesh);
    calcNorm.compute();
    _out_Mesh = calcNorm.getMesh();

    // Check Points and Normals
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (uint64_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        EXPECT_DOUBLE_EQ(out_Normal[0], _SavedPoints[pnt_id].nx);
        EXPECT_DOUBLE_EQ(out_Normal[1], _SavedPoints[pnt_id].ny);
        EXPECT_DOUBLE_EQ(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    EXPECT_EQ(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (uint64_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        EXPECT_EQ(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        EXPECT_EQ(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        EXPECT_EQ(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

TEST_F(QuadricArchFixture, QuadricResampledArchTest)
{
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells() / 2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm(_out_Mesh);
    calcNorm.compute();
    _out_Mesh = calcNorm.getMesh();

    // Check Points and Normals
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (uint64_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        EXPECT_DOUBLE_EQ(out_Normal[0], _SavedPoints[pnt_id].nx);
        EXPECT_DOUBLE_EQ(out_Normal[1], _SavedPoints[pnt_id].ny);
        EXPECT_DOUBLE_EQ(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    EXPECT_EQ(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (uint64_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        EXPECT_EQ(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        EXPECT_EQ(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        EXPECT_EQ(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

TEST_F(QuadricConeFixture, QuadricResampledConeTest)
{
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells() / 2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm(_out_Mesh);
    calcNorm.compute();
    _out_Mesh = calcNorm.getMesh();

    // Check Points and Normals
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (uint64_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        EXPECT_DOUBLE_EQ(out_Normal[0], _SavedPoints[pnt_id].nx);
        EXPECT_DOUBLE_EQ(out_Normal[1], _SavedPoints[pnt_id].ny);
        EXPECT_DOUBLE_EQ(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    EXPECT_EQ(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (uint64_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        EXPECT_EQ(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        EXPECT_EQ(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        EXPECT_EQ(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

TEST_F(QuadricCubeFixture, QuadricResampledCubeTest)
{
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells() / 2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm(_out_Mesh);
    calcNorm.compute();
    _out_Mesh = calcNorm.getMesh();

    // Check Points and Normals
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (uint64_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        EXPECT_DOUBLE_EQ(out_Normal[0], _SavedPoints[pnt_id].nx);
        EXPECT_DOUBLE_EQ(out_Normal[1], _SavedPoints[pnt_id].ny);
        EXPECT_DOUBLE_EQ(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    EXPECT_EQ(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (uint64_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        EXPECT_EQ(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        EXPECT_EQ(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        EXPECT_EQ(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

TEST_F(QuadricSphereFixture, QuadricResampledSphereTest)
{
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells() / 2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm(_out_Mesh);
    calcNorm.compute();
    _out_Mesh = calcNorm.getMesh();

    // Check Points and Normals
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (uint64_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        EXPECT_DOUBLE_EQ(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        EXPECT_DOUBLE_EQ(out_Normal[0], _SavedPoints[pnt_id].nx);
        EXPECT_DOUBLE_EQ(out_Normal[1], _SavedPoints[pnt_id].ny);
        EXPECT_DOUBLE_EQ(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    EXPECT_EQ(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (uint64_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        EXPECT_EQ(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        EXPECT_EQ(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        EXPECT_EQ(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}
