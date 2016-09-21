//
// Created by Hannah Hatch on 8/23/16.
//

#define BOOST_TEST_MODULE OrderedResampling

#include <boost/test/unit_test.hpp>
#include "common/io/objWriter.h"
#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"
#include "common/vc_defines.h"
#include "meshing/OrderedPointSetMesher.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"

using namespace volcart;

struct OrderedPlaneFixture {
    OrderedPlaneFixture() : _Plane(5)
    {
        _Plane.push_row({{0.0, 0.0, 0.0},
                         {0.0, 0.0, 2.0},
                         {0.0, 0.0, 4.0},
                         {0.0, 0.0, 6.0},
                         {0.0, 0.0, 8.0}});
        _Plane.push_row({{2.0, 0.0, 0.0},
                         {2.0, 0.0, 2.0},
                         {2.0, 0.0, 4.0},
                         {2.0, 0.0, 6.0},
                         {2.0, 0.0, 8.0}});
        _Plane.push_row({{4.0, 0.0, 0.0},
                         {4.0, 0.0, 2.0},
                         {4.0, 0.0, 4.0},
                         {4.0, 0.0, 6.0},
                         {4.0, 0.0, 8.0}});
        _Plane.push_row({{6.0, 0.0, 0.0},
                         {6.0, 0.0, 2.0},
                         {6.0, 0.0, 4.0},
                         {6.0, 0.0, 6.0},
                         {6.0, 0.0, 8.0}});
        _Plane.push_row({{8.0, 0.0, 0.0},
                         {8.0, 0.0, 2.0},
                         {8.0, 0.0, 4.0},
                         {8.0, 0.0, 6.0},
                         {8.0, 0.0, 8.0}});

        volcart::testing::ParsingHelpers::parseObjFile(
            "OrderedPointSetMesher_Plane.obj", _SavedPoints, _SavedCells);
    }
    OrderedPointSet<Point3d> _Plane;
    VC_MeshType::Pointer _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

struct OrderedArchFixture {
    OrderedArchFixture() : _Arch(5)
    {
        _Arch.push_row({{5.0, 0.0, 0.0},
                        {4.04508, 2.93893, 0.0},
                        {1.54508, 4.75528, 0.0},
                        {-1.54508, 4.75528, 0.0},
                        {-4.04508, 2.93893, 0.0}});
        _Arch.push_row({{5.0, 0.0, 2.0},
                        {4.04508, 2.93893, 2.0},
                        {1.54508, 4.75528, 2.0},
                        {-1.54508, 4.75528, 2.0},
                        {-4.04508, 2.93893, 2.0}});
        _Arch.push_row({{5.0, 0.0, 4.0},
                        {4.04508, 2.93893, 4.0},
                        {1.54508, 4.75528, 4.0},
                        {-1.54508, 4.75528, 4.0},
                        {-4.04508, 2.93893, 4.0}});
        _Arch.push_row({{5.0, 0.0, 6.0},
                        {4.04508, 2.93893, 6.0},
                        {1.54508, 4.75528, 6.0},
                        {-1.54508, 4.75528, 6.0},
                        {-4.04508, 2.93893, 6.0}});
        _Arch.push_row({{5.0, 0.0, 8.0},
                        {4.04508, 2.93893, 8.0},
                        {1.54508, 4.75528, 8.0},
                        {-1.54508, 4.75528, 8.0},
                        {-4.04508, 2.93893, 8.0}});

        volcart::testing::ParsingHelpers::parseObjFile(
            "OrderedPointSetMesher_Arch.obj", _SavedPoints, _SavedCells);
    }
    OrderedPointSet<Point3d> _Arch;
    VC_MeshType::Pointer _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

BOOST_FIXTURE_TEST_CASE(MeshedPlaneTest, OrderedPlaneFixture)
{
    volcart::meshing::OrderedPointSetMesher mesher_plane(_Plane);
    mesher_plane.compute();
    _out_Mesh = mesher_plane.getOutputMesh();

    // Check Points and Normals
    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (unsigned long pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        VC_PixelType out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (unsigned long cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        VC_CellType::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

BOOST_FIXTURE_TEST_CASE(MeshedArchTest, OrderedArchFixture)
{
    volcart::meshing::OrderedPointSetMesher mesher_arch(_Arch);
    mesher_arch.compute();
    _out_Mesh = mesher_arch.getOutputMesh();

    // Check Points and Normals
    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for (unsigned long pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        VC_PixelType out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for (unsigned long cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        VC_CellType::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}
