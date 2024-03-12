#include <gtest/gtest.h>

#include <cstddef>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYWriter.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/testing/ParsingHelpers.hpp"

using namespace volcart;
using namespace volcart::testing;

class PlaneFixture : public ::testing::Test
{
public:
    PlaneFixture()
    {
        volcart::shapes::Plane plane;
        _input = plane.itkMesh();
    }

    ITKMesh::Pointer _input;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

TEST_F(PlaneFixture, ParseOBJTest)
{

    // Write the mesh
    volcart::io::OBJWriter OBJWriter;
    OBJWriter.setMesh(_input);
    OBJWriter.setPath("ParsingHelpers_Plane.obj");
    OBJWriter.write();

    // Parse the mesh
    volcart::testing::ParsingHelpers::ParseOBJFile(
        "ParsingHelpers_Plane.obj", _SavedPoints, _SavedCells);

    // Check Points and Normals
    EXPECT_EQ(_input->GetNumberOfPoints(), _SavedPoints.size());
    for (std::size_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        EXPECT_DOUBLE_EQ(_input->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        EXPECT_DOUBLE_EQ(_input->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        EXPECT_DOUBLE_EQ(_input->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _input->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        EXPECT_DOUBLE_EQ(out_Normal[0], _SavedPoints[pnt_id].nx);
        EXPECT_DOUBLE_EQ(out_Normal[1], _SavedPoints[pnt_id].ny);
        EXPECT_DOUBLE_EQ(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    EXPECT_EQ(_input->GetNumberOfCells(), _SavedCells.size());
    for (std::size_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _input->GetCell(cell_id, current_C);
        EXPECT_EQ(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        EXPECT_EQ(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        EXPECT_EQ(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

TEST_F(PlaneFixture, ParsePLYTest)
{

    // Write the mesh
    volcart::io::PLYWriter PLYWriter;
    PLYWriter.setMesh(_input);
    PLYWriter.setPath("ParsingHelpers_Plane.ply");
    PLYWriter.write();

    // Parse the mesh
    volcart::testing::ParsingHelpers::ParsePLYFile(
        "ParsingHelpers_Plane.ply", _SavedPoints, _SavedCells);

    // Check Points and Normals
    EXPECT_EQ(_input->GetNumberOfPoints(), _SavedPoints.size());
    for (std::size_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        EXPECT_DOUBLE_EQ(_input->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        EXPECT_DOUBLE_EQ(_input->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        EXPECT_DOUBLE_EQ(_input->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _input->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        EXPECT_DOUBLE_EQ(out_Normal[0], _SavedPoints[pnt_id].nx);
        EXPECT_DOUBLE_EQ(out_Normal[1], _SavedPoints[pnt_id].ny);
        EXPECT_DOUBLE_EQ(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    EXPECT_EQ(_SavedCells.size(), _input->GetNumberOfCells());
    for (std::size_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _input->GetCell(cell_id, current_C);
        EXPECT_EQ(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        EXPECT_EQ(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        EXPECT_EQ(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}