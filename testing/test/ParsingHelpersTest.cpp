//
// Created by Seth Parker on 8/31/16.
//

#define BOOST_TEST_MODULE ParsingHelpers

#include <boost/test/unit_test.hpp>

#include "core/io/OBJWriter.h"
#include "core/io/PLYWriter.h"
#include "core/shapes/Plane.h"
#include "core/vc_defines.h"
#include "testing/ParsingHelpers.h"
#include "testing/TestingUtils.h"

using namespace volcart;
using namespace volcart::testing;

/************************************************************************************
 *                                                                                  *
 *  ParsingHelpersTest.cpp - tests the functionality of *
 *  /v-c/testing/src/parsinHelpers.cpp. The ultimate goal of this file is *
 *  the following: *
 *                                                                                  *
 *        1. check whether a parsed OBJ file matches the data structure that *
 *        generated it. *
 *                                                                                  *
 *        2. check whether a parsed PLY file matches the data structure that *
 *        generated it. *
 *                                                                                  *
 *     No required inputs for this sample test. All test objects are created *
 *     internally. *
 *                                                                                  *
 * Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general  *
 *     number of testing errors is output. *
 * **********************************************************************************/

struct PlaneFixture {

    PlaneFixture()
    {
        volcart::shapes::Plane plane;
        _input = plane.itkMesh();
    }

    ITKMesh::Pointer _input;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;
};

BOOST_FIXTURE_TEST_CASE(ParseOBJTest, PlaneFixture)
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
    BOOST_CHECK_EQUAL(_input->GetNumberOfPoints(), _SavedPoints.size());
    for (size_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        SmallOrClose(_input->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        SmallOrClose(_input->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        SmallOrClose(_input->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _input->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        SmallOrClose(out_Normal[0], _SavedPoints[pnt_id].nx);
        SmallOrClose(out_Normal[1], _SavedPoints[pnt_id].ny);
        SmallOrClose(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    BOOST_CHECK_EQUAL(_input->GetNumberOfCells(), _SavedCells.size());
    for (size_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _input->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}

BOOST_FIXTURE_TEST_CASE(ParsePLYTest, PlaneFixture)
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
    BOOST_CHECK_EQUAL(_input->GetNumberOfPoints(), _SavedPoints.size());
    for (size_t pnt_id = 0; pnt_id < _SavedPoints.size(); pnt_id++) {
        SmallOrClose(_input->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        SmallOrClose(_input->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        SmallOrClose(_input->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);

        ITKPixel out_Normal;
        _input->GetPointData(pnt_id, &out_Normal);

        // Now compare the normals for the two meshes
        SmallOrClose(out_Normal[0], _SavedPoints[pnt_id].nx);
        SmallOrClose(out_Normal[1], _SavedPoints[pnt_id].ny);
        SmallOrClose(out_Normal[2], _SavedPoints[pnt_id].nz);
    }

    // Check Cells, Checks Point normals by ensuring that the first vertex is
    // the same in both
    BOOST_CHECK_EQUAL(_SavedCells.size(), _input->GetNumberOfCells());
    for (size_t cell_id = 0; cell_id < _SavedCells.size(); cell_id++) {
        ITKCell::CellAutoPointer current_C;
        _input->GetCell(cell_id, current_C);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[0], _SavedCells[cell_id].v1);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[1], _SavedCells[cell_id].v2);
        BOOST_CHECK_EQUAL(current_C->GetPointIds()[2], _SavedCells[cell_id].v3);
    }
}
