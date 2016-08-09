//
// Created by Ryan Taber on 9/25/15.
//

#define BOOST_TEST_MODULE ObjWriter

#include <boost/test/unit_test.hpp>
#include "common/shapes/Plane.h"
#include "common/io/objWriter.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"


/***************************************************************************************
 *                                                                                     *
 *  ObjWriterTest.cpp - tests the functionality of /v-c/common/io/objWriter.cpp        *
 *  The ultimate goal of this file is the following:                                   *
 *                                                                                     *
 *        1. check whether a mesh, created by common/shapes/Plane.h,                   *
 *           can be written into obj file by common/io/objWriter.cpp. This test is     *
 *           implicit in the ObjWriterExample.cpp.                                     *
 *                                                                                     *
 *        2. read contents of obj file and compare data with testing mesh              *
 *                                                                                     *
 *  This file is broken up into a testing fixture which initializes the objects used   *
 *  in the test case:                                                                  *
 *                                                                                     *
 *  1. CompareWrittenMeshDataWithFixtureCreatedMesh                                    *
 *      Read in saved data from OBJWriterPlaneData.obj and compare points, normals,    *
 *      and cells with the fixture created mesh, _in_PlaneITKMesh.                     *
 *                                                                                     *
 *  Input:                                                                             *
 *     No required inputs for this sample test. Note: the output.obj must be copied    *
 *     from test_data/common to curr_bin_dir when building, which is handled by cmake. *
 *                                                                                     *
 *  Test-Specific Output:                                                              *
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

        volcart::testing::SmallOrClose(_in_PlaneITKMesh->GetPoint(pnt)[0], _SavedPlanePoints[pnt].x);
        volcart::testing::SmallOrClose(_in_PlaneITKMesh->GetPoint(pnt)[1], _SavedPlanePoints[pnt].y);
        volcart::testing::SmallOrClose(_in_PlaneITKMesh->GetPoint(pnt)[2], _SavedPlanePoints[pnt].z);
    }

    // Normals //
    int p =0;
    VC_PointsInMeshIterator point = _in_PlaneITKMesh->GetPoints()->Begin();
    for ( ; point != _in_PlaneITKMesh->GetPoints()->End(); ++point ) {

        VC_PixelType _in_PlaneITKMeshNormal;
        _in_PlaneITKMesh->GetPointData(point.Index(), &_in_PlaneITKMeshNormal);

        //Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(_in_PlaneITKMeshNormal[0], _SavedPlanePoints[p].nx);
        volcart::testing::SmallOrClose(_in_PlaneITKMeshNormal[1], _SavedPlanePoints[p].ny);
        volcart::testing::SmallOrClose(_in_PlaneITKMeshNormal[2], _SavedPlanePoints[p].nz);

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


