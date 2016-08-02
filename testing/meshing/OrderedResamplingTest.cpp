//
// Created by Hannah Hatch on 7/26/16.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE OrderedResampling

#include <boost/test/unit_test.hpp>
#include "vc_defines.h"
#include "OrderedResampling.h"
#include "parsingHelpers.h"
#include "testingUtils.h"
#include "shapes.h"

struct OrderedPlaneFixture {
    OrderedPlaneFixture(){
        _Plane = volcart::shapes::Plane(10, 10);
        _in_Mesh = _Plane.itkMesh();
        _in_height = _Plane.orderedHeight();
        _in_width = _Plane.orderedWidth();
        volcart::testing::ParsingHelpers::parseObjFile("OrderedResampling_Plane.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Plane _Plane;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
    int _in_height, _in_width;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

struct OrderedArchFixture {
    OrderedArchFixture(){
        _Arch = volcart::shapes::Arch(20, 20);
        _in_Mesh = _Arch.itkMesh();
        _in_height = _Arch.orderedHeight();
        _in_width = _Arch.orderedWidth();
        volcart::testing::ParsingHelpers::parseObjFile("OrderedResampling_Arch.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Arch _Arch;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
    int _in_height, _in_width;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

BOOST_FIXTURE_TEST_CASE(ResampledPlaneTest, OrderedPlaneFixture){
    volcart::meshing::OrderedResampling resample(_in_Mesh, _in_width, _in_height);
    resample.compute();
    _out_Mesh = resample.getOutputMesh();

    //Check Points
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    for(unsigned long pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++)
    {
        volcart::testing::SmallOrClose(_SavedPoints[pnt_id].x, _out_Mesh->GetPoint(pnt_id)[0]);
        volcart::testing::SmallOrClose(_SavedPoints[pnt_id].y, _out_Mesh->GetPoint(pnt_id)[1]);
        volcart::testing::SmallOrClose(_SavedPoints[pnt_id].z, _out_Mesh->GetPoint(pnt_id)[2]);
    }

    //Check Cells, Checks Point normals by ensuring that the first vertex is the same in both
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for(unsigned long cell_id = 0; cell_id < _SavedCells.size(); cell_id++)
    {
        VC_CellType::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(_SavedCells[cell_id].v1, current_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(_SavedCells[cell_id].v2, current_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(_SavedCells[cell_id].v3, current_C->GetPointIds()[2]);
    }
}

BOOST_FIXTURE_TEST_CASE(ResampledArchTest, OrderedArchFixture){
    volcart::meshing::OrderedResampling resample(_in_Mesh, _in_width, _in_height);
    resample.compute();
    _out_Mesh = resample.getOutputMesh();

    //Check Points
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    for(unsigned long pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++)
    {
        volcart::testing::SmallOrClose(_SavedPoints[pnt_id].x, _out_Mesh->GetPoint(pnt_id)[0]);
        volcart::testing::SmallOrClose(_SavedPoints[pnt_id].y, _out_Mesh->GetPoint(pnt_id)[1]);
        volcart::testing::SmallOrClose(_SavedPoints[pnt_id].z, _out_Mesh->GetPoint(pnt_id)[2]);
    }

    //Check Cells, Checks Point normals by ensuring that the first vertex is the same in both
    BOOST_CHECK_EQUAL(_SavedCells.size(), _out_Mesh->GetNumberOfCells());
    for(unsigned long cell_id = 0; cell_id < _SavedCells.size(); cell_id++)
    {
        VC_CellType::CellAutoPointer current_C;
        _out_Mesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(_SavedCells[cell_id].v1, current_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(_SavedCells[cell_id].v2, current_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(_SavedCells[cell_id].v3, current_C->GetPointIds()[2]);
    }
}