//
// Created by Hannah Hatch on 8/17/16.
//

#define BOOST_TEST_MODULE QuadricEdgeCollapseDecimation

#include <boost/test/unit_test.hpp>
#include "common/vc_defines.h"
#include "meshing/QuadricEdgeCollapseDecimation.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Arch.h"
#include "common/shapes/Cone.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Sphere.h"
#include "meshing/CalculateNormals.h"


/************************************************************************************
 *                                                                                  *
 *  QuardricEdgeCollapseDecimation.cpp - tests the functionality of                 *
 *      /vc/meshing/QuadricEdgeCollapseDecimation.cpp                               *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether an itk mesh can be converted to a vcg mesh               *
 *           and vice versa.                                                        *
 *                                                                                  *
 *        2. confirm that the decimation algorithm creates files that match         *
 *            generated obj files                                                   *
 *  This file is broken up into 5 test fixtures (Plane,Arch,Cube,Cone,Sphere)       *
 *  which initialize the objects used in each of the five test cases.               *
 *                                                                                  *
 *  Each functions similarly by creating a new object of a particular shape,        *
 *  running the decimation algorithm which converts it to vcg and back to itk       *
 *  then check them agaist previously rendered files                                *
 *                                                                                  *
 * Input:                                                                           *
 *     No required inputs for this sample test. All test objects are created        *
 *     internally.                                                                  *
 *                                                                                  *
 * Test-Specific Output:                                                            *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output.                                          *
 *                                                                                  *
 * Miscellaneous:                                                                   *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/



//Setting up the shapes
struct QuadricPlaneFixture {
    QuadricPlaneFixture(){
        _Plane = volcart::shapes::Plane(10, 10);
        _in_Mesh = _Plane.itkMesh();
        volcart::testing::ParsingHelpers::parseObjFile("QuadricEdgeCollapse_Plane.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Plane _Plane;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

struct QuadricArchFixture {
    QuadricArchFixture(){
        _Arch = volcart::shapes::Arch(100, 100);
        _in_Mesh = _Arch.itkMesh();
        volcart::testing::ParsingHelpers::parseObjFile("QuadricEdgeCollapse_Arch.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Arch _Arch;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

struct QuadricConeFixture {
    QuadricConeFixture(){
        _Cone = volcart::shapes::Cone(1000, 1000);
        _in_Mesh = _Cone.itkMesh();
        volcart::testing::ParsingHelpers::parseObjFile("QuadricEdgeCollapse_Cone.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Cone _Cone;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

struct QuadricCubeFixture {
    QuadricCubeFixture(){
        _Cube = volcart::shapes::Cube();
        _in_Mesh = _Cube.itkMesh();
        volcart::testing::ParsingHelpers::parseObjFile("QuadricEdgeCollapse_Cube.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Cube _Cube;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};

struct QuadricSphereFixture {
    QuadricSphereFixture(){
        _Sphere = volcart::shapes::Sphere(30, 3);
        _in_Mesh = _Sphere.itkMesh();
        volcart::testing::ParsingHelpers::parseObjFile("QuadricEdgeCollapse_Sphere.obj", _SavedPoints, _SavedCells);
    }
    volcart::shapes::Sphere _Sphere;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
    std::vector<VC_Vertex> _SavedPoints;
    std::vector<VC_Cell> _SavedCells;
};


//Tests by Shape
BOOST_FIXTURE_TEST_CASE(QuadricResampledPlaneTest, QuadricPlaneFixture){
    volcart::meshing::QuadricEdgeCollapseDecimation resample;
    resample.setMesh(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells()/2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm( _out_Mesh );
    calcNorm.compute();
    _out_Mesh= calcNorm.getMesh();

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

BOOST_FIXTURE_TEST_CASE(QuadricResampledArchTest, QuadricArchFixture){
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells()/2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm( _out_Mesh );
    calcNorm.compute();
    _out_Mesh= calcNorm.getMesh();

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

BOOST_FIXTURE_TEST_CASE(QuadricResampledConeTest, QuadricConeFixture){
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells()/2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm( _out_Mesh );
    calcNorm.compute();
    _out_Mesh= calcNorm.getMesh();

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

BOOST_FIXTURE_TEST_CASE(QuadricResampledCubeTest, QuadricCubeFixture){
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells()/2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm( _out_Mesh );
    calcNorm.compute();
    _out_Mesh= calcNorm.getMesh();

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

BOOST_FIXTURE_TEST_CASE(QuadricResampledSphereTest, QuadricSphereFixture){
    volcart::meshing::QuadricEdgeCollapseDecimation resample(_in_Mesh);
    resample.compute(_in_Mesh->GetNumberOfCells()/2);
    _out_Mesh = resample.getMesh();
    volcart::meshing::CalculateNormals calcNorm( _out_Mesh );
    calcNorm.compute();
    _out_Mesh= calcNorm.getMesh();

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