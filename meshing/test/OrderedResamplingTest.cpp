//
// Created by Hannah Hatch on 7/26/16.
//

#define BOOST_TEST_MODULE OrderedResampling

#include <boost/test/unit_test.hpp>
#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/meshing/OrderedResampling.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;

struct OrderedPlaneFixture {
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

struct OrderedArchFixture {
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

BOOST_FIXTURE_TEST_CASE(ResampledPlaneTest, OrderedPlaneFixture)
{
    volcart::meshing::OrderedResampling resample(
        _in_Mesh, _in_width, _in_height);
    resample.compute();
    _out_Mesh = resample.getOutputMesh();

    // Check Points and Normals
    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
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
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (uint64_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

BOOST_FIXTURE_TEST_CASE(ResampledArchTest, OrderedArchFixture)
{
    volcart::meshing::OrderedResampling resample(
        _in_Mesh, _in_width, _in_height);
    resample.compute();
    _out_Mesh = resample.getOutputMesh();

    // Check Points and Normals
    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
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
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (uint64_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}
