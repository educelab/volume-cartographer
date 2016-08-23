//
// Created by Hannah Hatch on 8/23/16.
//

#define BOOST_TEST_MODULE OrderedResampling

#include <boost/test/unit_test.hpp>
#include "common/vc_defines.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"
#include "common/types/Point.h"
#include "common/types/PointSet.h"
#include "common/io/objWriter.h"
#include "meshing/OrderedPointSetMesher.h"

using namespace volcart;

struct OrderedPlaneFixture {
    OrderedPlaneFixture(): _Plane(5,5){
        _Plane.push_row({{0.0, 0.0, 0.0},{0.0, 0.0, 2.0}, {0.0, 0.0, 4.0}, {0.0, 0.0, 6.0}, {0.0, 0.0, 8.0}});
        _Plane.push_row({{2.0, 0.0, 0.0},{2.0, 0.0, 2.0}, {2.0, 0.0, 4.0}, {2.0, 0.0, 6.0}, {2.0, 0.0, 8.0}});
        _Plane.push_row({{4.0, 0.0, 0.0},{4.0, 0.0, 2.0}, {4.0, 0.0, 4.0}, {4.0, 0.0, 6.0}, {4.0, 0.0, 8.0}});
        _Plane.push_row({{6.0, 0.0, 0.0},{6.0, 0.0, 2.0}, {6.0, 0.0, 4.0}, {6.0, 0.0, 6.0}, {6.0, 0.0, 8.0}});
        _Plane.push_row({{8.0, 0.0, 0.0},{8.0, 0.0, 2.0}, {8.0, 0.0, 4.0}, {8.0, 0.0, 6.0}, {8.0, 0.0, 8.0}});

        volcart::testing::ParsingHelpers::parseObjFile("OrderedPointSetMesher_Plane.obj", _SavedPoints, _SavedCells);
    }
    PointSet<Point3d> _Plane;
    VC_MeshType::Pointer _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

struct OrderedArchFixture {
    OrderedArchFixture(): _Arch(7,7){
        _Arch.push_row({{5.0, 0.0, 0.0},{4.69846, 1.7101, 0.0}, {3.83022, 3.21394, 0.0}, {2.5, 4.33013, 0.0}, {0.868241, 4.92404, 0.0},{-0.868241, 4.92404, 0.0},{-2.5, 4.33013, 0.0}});
        _Arch.push_row({{-3.83022, 3.21394, 0.0},{-4.69846, 1.7101, 0.0}, {5.0, 0.0, 1.0}, {4.69846, 1.7101, 1.0}, {3.83022, 3.21394, 1.0},{-4.69846, 1.7101, 1.0},{5.0, 0.0, 2.0}});
        _Arch.push_row({{4.69846, 1.7101, 5.0},{3.83022, 3.21394, 3.625}, {2.5, 4.33013, 0.0}, {-4.69846, 1.7101, 2.0}, {5.0, 0.0, 3.0},{-0.868241, 4.92404, 1.75},{-2.5, 4.33013, 1.75}});
        _Arch.push_row({{-3.83022, 3.21394, 1.75},{-4.69846, 1.7101, 3.0}, {5.0, 0.0, 4.0}, {-4.69846, 1.7101, 4.0}, {5.0, 0.0, 5.0},{2.5, 4.33013, 5.0},{-0.868241, 4.92404, 4.5}});
        _Arch.push_row({{-2.5, 4.33013, 5.5},{-3.83022, 3.21394, 4.5}, {-4.69846, 1.7101, 5.0}, {5.0, 0.0, 6.0}, {-4.69846, 1.7101, 6.0},{5.0, 0.0, 7.0},{3.83022, 3.21394, 7.0}});
        _Arch.push_row({{2.5, 4.33013, 0.0},{0.868241, 4.92404, 4.4375}, {-0.868241, 4.92404, 6.5}, {-3.83022, 3.21394, 6.5}, { -4.69846, 1.7101, 7.0},{5.0, 0.0, 8.0},{4.69846, 1.7101, 8.0}});
        _Arch.push_row({{3.83022, 3.21394, 8.0},{2.5, 4.33013, 8.0}, {0.868241, 4.92404, 8.0}, {-0.868241, 4.92404, 8.0}, {-2.5, 4.33013, 8.0},{-3.83022, 3.21394, 8.0},{-4.69846, 1.7101, 8.0}});



        volcart::testing::ParsingHelpers::parseObjFile("OrderedPointSetMesher_Arch.obj", _SavedPoints, _SavedCells);
    }
    PointSet<Point3d> _Arch;
    VC_MeshType::Pointer _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

BOOST_FIXTURE_TEST_CASE(MeshedPlaneTest, OrderedPlaneFixture){
    volcart::meshing::OrderedPointSetMesher mesher_plane(_Plane);
    mesher_plane.compute();
    _out_Mesh = mesher_plane.getOutput();

    //Check Points and Normals
    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for(unsigned long pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++)
    {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        VC_PixelType out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        //Now compare the normals for the two meshes
        volcart::testing::SmallOrClose( out_Normal[0], _SavedPoints[pnt_id].nx );
        volcart::testing::SmallOrClose( out_Normal[1], _SavedPoints[pnt_id].ny );
        volcart::testing::SmallOrClose( out_Normal[2], _SavedPoints[pnt_id].nz );
    }

    //Check Cells, Checks Point normals by ensuring that the first vertex is the same in both
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for(unsigned long cell_id = 0; cell_id < _SavedCells.size(); cell_id++)
    {
        VC_CellType::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

BOOST_FIXTURE_TEST_CASE(MeshedArchTest, OrderedArchFixture){
    volcart::meshing::OrderedPointSetMesher mesher_arch(_Arch);
    mesher_arch.compute();
    _out_Mesh = mesher_arch.getOutput();

    //Check Points and Normals
    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());
    for(unsigned long pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++)
    {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        VC_PixelType out_Normal;
        _out_Mesh->GetPointData(pnt_id, &out_Normal);

        //Now compare the normals for the two meshes
        volcart::testing::SmallOrClose( out_Normal[0], _SavedPoints[pnt_id].nx );
        volcart::testing::SmallOrClose( out_Normal[1], _SavedPoints[pnt_id].ny );
        volcart::testing::SmallOrClose( out_Normal[2], _SavedPoints[pnt_id].nz );
    }

    //Check Cells, Checks Point normals by ensuring that the first vertex is the same in both
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for(unsigned long cell_id = 0; cell_id < _SavedCells.size(); cell_id++)
    {
        VC_CellType::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}