//
// Created by Hannah Hatch on 7/26/16.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE orderedResampling

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "orderedResampling.h"
#include "parsingHelpers.h"
#include "shapes.h"

struct orderedPlaneFixture {
    orderedPlaneFixture(){
        _in_PlaneMesh = _Plane.itkMesh();
        plane_height = _Plane.orderedHeight();
        plane_width = _Plane.orderedWidth();
        volcart::testing::ParsingHelpers::parseObjFile("PlaneOrderedResampling.obj", _SavedPlanePoints, _SavedPlaneCells);
    }
    volcart::shapes::Plane _Plane;
    VC_MeshType::Pointer _in_PlaneMesh, _out_PlaneMesh;
    int plane_height, plane_width;
    std::vector<VC_Vertex> _SavedPlanePoints;
    std::vector<VC_Cell> _SavedPlaneCells;
};

struct orderedArchFixture {
    orderedArchFixture(){
        _in_ArchMesh = _Arch.itkMesh();
        arch_height = _Arch.orderedHeight();
        arch_width = _Arch.orderedWidth();
        volcart::testing::ParsingHelpers::parseObjFile("ArchOrderedResampling.obj", _SavedArchPoints, _SavedArchCells);
    }
    volcart::shapes::Arch _Arch;
    VC_MeshType::Pointer _in_ArchMesh, _out_ArchMesh;
    int arch_height, arch_width;
    std::vector<VC_Vertex> _SavedArchPoints;
    std::vector<VC_Cell> _SavedArchCells;
};

BOOST_FIXTURE_TEST_CASE(ResampledPlaneTest, orderedPlaneFixture){
    volcart::meshing::orderedResampling resamplePlane(_in_PlaneMesh, plane_width, plane_height);
    resamplePlane.compute();
    _out_PlaneMesh = resamplePlane.getOutputMesh();

    //Check Points
    BOOST_CHECK_EQUAL(_SavedPlanePoints.size(), _out_PlaneMesh->GetNumberOfPoints());

    for(int pnt_id = 0; pnt_id < _SavedPlanePoints.size() - 1; pnt_id++)
    {
        BOOST_CHECK_EQUAL(_SavedPlanePoints[pnt_id].x, _out_PlaneMesh->GetPoint(pnt_id)[0]);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[pnt_id].y, _out_PlaneMesh->GetPoint(pnt_id)[1]);
        BOOST_CHECK_EQUAL(_SavedPlanePoints[pnt_id].z, _out_PlaneMesh->GetPoint(pnt_id)[2]);

    }


    //Check Cells, Checks Point normals by ensuring that the first vertex is the same in both
    BOOST_CHECK_EQUAL(_SavedPlaneCells.size(), _out_PlaneMesh->GetNumberOfCells());
    for(unsigned long cell_id = 0; cell_id < _SavedPlaneCells.size(); cell_id++)
    {
        VC_CellType::CellAutoPointer current_C;
        _out_PlaneMesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(_SavedPlaneCells[cell_id].v1, current_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(_SavedPlaneCells[cell_id].v2, current_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(_SavedPlaneCells[cell_id].v3, current_C->GetPointIds()[2]);

    }



}

BOOST_FIXTURE_TEST_CASE(ResampledArchTest, orderedArchFixture){
    volcart::meshing::orderedResampling resampleArch(_in_ArchMesh, arch_width, arch_height);
    resampleArch.compute();
    _out_ArchMesh = resampleArch.getOutputMesh();

    //Check Points
    BOOST_CHECK_EQUAL(_SavedArchPoints.size(), _out_ArchMesh->GetNumberOfPoints());

    for(int pnt_id = 0; pnt_id < _SavedArchPoints.size() -1; pnt_id++)
    {
        BOOST_CHECK_EQUAL(_SavedArchPoints[pnt_id].x, _out_ArchMesh->GetPoint(pnt_id)[0]);
        BOOST_CHECK_EQUAL(_SavedArchPoints[pnt_id].y, _out_ArchMesh->GetPoint(pnt_id)[1]);
        BOOST_CHECK_EQUAL(_SavedArchPoints[pnt_id].z, _out_ArchMesh->GetPoint(pnt_id)[2]);
    }


    //Check Cells, Checks Point normals by ensuring that the first vertex is the same in both
    BOOST_CHECK_EQUAL(_SavedArchCells.size(), _out_ArchMesh->GetNumberOfCells());
    for(unsigned long cell_id = 0; cell_id < _SavedArchCells.size(); cell_id++)
    {
        VC_CellType::CellAutoPointer current_C;
        _out_ArchMesh->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(_SavedArchCells[cell_id].v1, current_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(_SavedArchCells[cell_id].v2, current_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(_SavedArchCells[cell_id].v3, current_C->GetPointIds()[2]);

    }



}