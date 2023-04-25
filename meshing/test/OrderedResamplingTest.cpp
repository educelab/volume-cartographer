#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/meshing/OrderedResampling.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;

class OrderedPlaneFixture : public ::testing::Test
{
public:
    OrderedPlaneFixture()
    {
        _Plane = volcart::shapes::Plane(10, 10);
        _in_Mesh = _Plane.itkMesh();
        _in_height = _Plane.orderedHeight();
        _in_width = _Plane.orderedWidth();
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "OrderedResampling_Plane.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Plane _Plane;
    ITKMesh::Pointer _in_Mesh, _out_Mesh;
    int _in_height, _in_width;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class OrderedArchFixture : public ::testing::Test
{
public:
    OrderedArchFixture()
    {
        _Arch = volcart::shapes::Arch(20, 20);
        _in_Mesh = _Arch.itkMesh();
        _in_height = _Arch.orderedHeight();
        _in_width = _Arch.orderedWidth();
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "OrderedResampling_Arch.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_Mesh, _out_Mesh;
    int _in_height, _in_width;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

TEST_F(OrderedPlaneFixture, ResampledPlaneTest)
{
    volcart::meshing::OrderedResampling resample(
        _in_Mesh, _in_width, _in_height);
    resample.compute();
    _out_Mesh = resample.getOutputMesh();

    // Check Points and Normals
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (uint64_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedPoints[pnt_id].nz);
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

TEST_F(OrderedArchFixture, ResampledArchTest)
{
    volcart::meshing::OrderedResampling resample(
        _in_Mesh, _in_width, _in_height);
    resample.compute();
    _out_Mesh = resample.getOutputMesh();

    // Check Points and Normals
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (uint64_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedPoints[pnt_id].nz);
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