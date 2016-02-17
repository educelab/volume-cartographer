//
// Created by Ryan Taber on 9/25/15.
//

#ifndef VC_PREBUILT_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE ObjWriter

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "shapes.h"
#include "vc_defines.h"
#include "io/objWriter.h"
#include "parsingHelpers.h"


/***************************************************************************************
 *                                                                                     *
 *  ObjWriterTest.cpp - tests the functionality of /v-c/common/io/objWriter.cpp        *
 *  The ultimate goal of this file is the following:                                   *
 *                                                                                     *
 *        1. check whether a testing mesh, created by                                  *
 *           common/shapes/Plane.h, can be written into                                *
 *           an object file by common/io/objWriter.cpp.                                *
 *                                                                                     *
 *        2. read contents of obj file and compare data with testing mesh              *
 *                                                                                     *
 *  This file is broken up into a testing fixture, meshFix, which initializes the      *
 *  objects used in each of the two test cases.                                        *
 *                                                                                     *
 *  writeTest (test case):                                                             *
 *                                                                                     *
 *      attempts to write a testing mesh to file and compares final output path        *
 *      as a check for success. Note, this test always outputs the file as             *
 *      "output.obj" because there is no texture information included when writing.    *
 *                                                                                     *
 *  compareElements (test case):                                                       *
 *                                                                                     *
 *      Attempts to read in information from ply file using created by objWriter.cpp.  *
 *      As data is read in from the ply file, collections of points and faces are      *
 *      created to compare against points and faces created during initialization      *
 *      of the testing mesh by meshFix. This parsing is done by the parsePly method    *
 *      implemented in parsingHelpers.cpp. The test then compares all non-commented    *
 *      data from the file against the testing mesh data to ensure equality.           *
 *                                                                                     *
 * Input:                                                                              *
 *     No required inputs for this sample test. Note: the output.obj must be copied    *
 *     from test_data/common to curr_bin_dir when building, which is handled by cmake. *
 *                                                                                     *
 * Test-Specific Output:                                                               *
 *     Specific test output only given on failure of any tests. Otherwise, general     *
 *     number of testing errors is output.                                             *
 *                                                                                     *
 * *************************************************************************************/



/*
 * This fixture builds objects for each of the test cases below that reference
 * the fixture as their second argument
 *
 */

struct WriteMeshUsingOBJWriterFixture {

    WriteMeshUsingOBJWriterFixture() {

        //create the mesh for all the test cases to use
        _in_PlaneITKMesh = _Plane.itkMesh();

        //load in written data
        volcart::testing::ParsingHelpers::parseObjFile("OBJWriterPlaneData.obj", _SavedPlanePoints, _SavedPlaneCells);

        std::cerr << "Creating a Plane itk mesh object for writing" << std::endl;
    }

    ~WriteMeshUsingOBJWriterFixture(){ std::cerr << "Cleaning up test objects" << std::endl; }

    volcart::shapes::Plane _Plane;
    VC_MeshType::Pointer _in_PlaneITKMesh ;
    std::vector<VC_Vertex>_SavedPlanePoints;
    std::vector<VC_Cell> _SavedPlaneCells;
};

BOOST_FIXTURE_TEST_CASE(CompareWrittenMeshDataWithFixtureCreatedMesh, WriteMeshUsingOBJWriterFixture){

    //compare number of points and cells for equality
    BOOST_CHECK_EQUAL(_in_PlaneITKMesh->GetNumberOfPoints(), _SavedPlanePoints.size());
    BOOST_CHECK_EQUAL(_in_PlaneITKMesh->GetNumberOfCells(), _SavedPlaneCells.size());
    
    // Now to test the objPoints created during fixture init vs points read in from file.
    for (size_t pnt = 0; pnt < _SavedPlanePoints.size(); pnt++) {

        //checking the x,y,z,nx,ny,nz components

        BOOST_CHECK_EQUAL(_in_PlaneITKMesh->GetPoint(pnt)[0], _SavedPlanePoints[pnt].x);
        BOOST_CHECK_EQUAL(_in_PlaneITKMesh->GetPoint(pnt)[1], _SavedPlanePoints[pnt].y);
        BOOST_CHECK_EQUAL(_in_PlaneITKMesh->GetPoint(pnt)[2], _SavedPlanePoints[pnt].z);
    }

    // Normals //
    int p =0;
    VC_PointsInMeshIterator point = _in_PlaneITKMesh->GetPoints()->Begin();
    for ( ; point != _in_PlaneITKMesh->GetPoints()->End(); ++point ) {

        VC_PixelType _in_PlaneITKMeshNormal;
        _in_PlaneITKMesh->GetPointData(point.Index(), &_in_PlaneITKMeshNormal);

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(_in_PlaneITKMeshNormal[0], _SavedPlanePoints[p].nx);
        BOOST_CHECK_EQUAL(_in_PlaneITKMeshNormal[1], _SavedPlanePoints[p].ny);
        BOOST_CHECK_EQUAL(_in_PlaneITKMeshNormal[2], _SavedPlanePoints[p].nz);

        ++p;
    }

    //Cells (faces)

    // Initialize Cell Iterators
    VC_CellIterator _in_PlaneITKMeshCell = _in_PlaneITKMesh->GetCells()->Begin();

    int c = 0;

    while (_in_PlaneITKMeshCell != _in_PlaneITKMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator _in_PlaneITKMeshPointId = _in_PlaneITKMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( _in_PlaneITKMeshPointId != _in_PlaneITKMeshCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*_in_PlaneITKMeshPointId, _SavedPlaneCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_in_PlaneITKMeshPointId, _SavedPlaneCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_in_PlaneITKMeshPointId, _SavedPlaneCells[c].v3);

            //increment points
            _in_PlaneITKMeshPointId++;
            counter++;

        }

        //increment cells
        ++_in_PlaneITKMeshCell;
        ++c;
    }

}


