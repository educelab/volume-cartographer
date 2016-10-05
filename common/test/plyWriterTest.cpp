//
// Created by Ryan Taber on 12/9/15.
//

#define BOOST_TEST_MODULE plyWriter

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "common/vc_defines.h"
#include "common/io/plyWriter.h"
#include "common/shapes/Plane.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"

using namespace volcart;

/***************************************************************************************
 *                                                                                     *
 *  plyWriterTest.cpp - tests the functionality of /v-c/common/plyWriter.cpp           *
 *  The ultimate goal of this file is the following:                                   *
 *                                                                                     *
 *        1. check whether a testing mesh, created by                                  *
 *           common/shapes/Plane.h, can be written into                                *
 *           a .ply file by common/io/plyWriter.cpp.                                   *
 *                                                                                     *
 *        2. read contents of ply file and compare data with testing mesh              *
 *                                                                                     *
 *  This file is broken up into a testing fixture, meshFix, which initializes the      *
 *  objects used in each of the two test cases.                                        *
 *                                                                                     *
 *  writeTest (test case):                                                             *
 *                                                                                     *
 *      attempts to write a testing mesh to file and compares final output path        *
 *      as a check for success. Note, this test always outputs the file as             *
 *      "PlyWriter_Plane.ply" because there is no texture information included when    *
 *      writing.                                                                       *
 *                                                                                     *
 *  compareElements (test case):                                                       *
 *                                                                                     *
 *      Attempts to read in information from ply file using created by plyWriter.cpp.  *
 *      As data is read in from the ply file, collections of points and faces are      *
 *      created to compare against points and faces created during initialization      *
 *      of the testing mesh by meshFix. This parsing is done by the parsePly method    *
 *      implemented in parsingHelpers.cpp. The test then compares all non-commented    *
 *      data from the file against the testing mesh data to ensure equality.           *
 *                                                                                     *
 * Input:                                                                              *
 *     No required inputs for this sample test. Note: the PlyWriter_Plane.ply must be  * 
 *     copied from test_data/common to curr_bin_dir when building, which is handled    *
 *     by cmake.                                                                       *
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

struct CreateITKPlaneMeshFixture {

  CreateITKPlaneMeshFixture() {

      //create the mesh for all the test cases to use
      _in_PlaneMesh = _Plane.itkMesh();


      //read in data from saved PlyWriter_Plane.ply via parsingHelpers::parsePlyFile
      volcart::testing::ParsingHelpers::parsePlyFile("PlyWriter_Plane.ply", _SavedPlanePoints, _SavedPlaneCells);
      
      BOOST_TEST_MESSAGE("setting up Plane mesh...");
  }

  ~CreateITKPlaneMeshFixture(){ BOOST_TEST_MESSAGE("cleaning up Plane mesh objects..."); }

  //file path and plyWriter to be used in cases
  volcart::io::plyWriter _MeshWriter;
  boost::filesystem::path ObjPath;
  ITKMesh::Pointer _in_PlaneMesh ;
  volcart::shapes::Plane _Plane;

  //vectors to hold vertices and faces saved in PlyWriter_Plane.ply
  std::vector<Vertex> _SavedPlanePoints;
  std::vector<Cell> _SavedPlaneCells;
};


// Test for checking successful write
BOOST_FIXTURE_TEST_CASE(WriteMeshToPLYFileTest, CreateITKPlaneMeshFixture) {

  std::cout << "Writing mesh to ply file..." << std::endl;

  _MeshWriter.setPath("nothing");
  //_MeshWriter.setUVMap( uvMap );
  // _MeshWriter.setTexture( uvImg );

  ObjPath = _MeshWriter.getPath();
  ObjPath = boost::filesystem::absolute(ObjPath);


  // _MeshWriter.write() runs validate() as well, but this lets us handle a mesh that can't be validated.
  if (_MeshWriter.validate())
      _MeshWriter.write();
  else {
      _MeshWriter.setPath("PlyWriter_Plane.ply");
      _MeshWriter.setMesh(_in_PlaneMesh);
      _MeshWriter.write();
  }

  //check the file path from the _MeshWriter.write() call above
  //compare() returns 0 only if paths are same value lexicographically-speaking
  //checking "PlyWriter_Plane.ply" here because the _in_PlaneMeshWriter shouldn't validate in the current case
  BOOST_CHECK_EQUAL(_MeshWriter.getPath().compare("PlyWriter_Plane.ply"), 0);

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureMeshAndSavedMeshData, CreateITKPlaneMeshFixture){

    //compare number of points and cells in each mesh
    BOOST_CHECK_EQUAL(_in_PlaneMesh->GetNumberOfPoints(), _SavedPlanePoints.size());
    BOOST_CHECK_EQUAL(_in_PlaneMesh->GetNumberOfCells(), _SavedPlaneCells.size());

    /// Points ///
    std::cerr << "Comparing points..." << std::endl;
    for (unsigned long p = 0; p < _SavedPlanePoints.size(); p++){

        volcart::testing::SmallOrClose(_SavedPlanePoints[p].x, _in_PlaneMesh->GetPoint(p)[0]);
        volcart::testing::SmallOrClose(_SavedPlanePoints[p].y, _in_PlaneMesh->GetPoint(p)[1]);
        volcart::testing::SmallOrClose(_SavedPlanePoints[p].z, _in_PlaneMesh->GetPoint(p)[2]);
    }

    // Normals //
    std::cerr << "Comparing normals..." << std::endl;
    int p_id = 0;
    for ( ITKPointIterator point = _in_PlaneMesh->GetPoints()->Begin(); point != _in_PlaneMesh->GetPoints()->End(); ++point ) {

        ITKPixel _in_PlaneMeshNormal;
        _in_PlaneMesh->GetPointData(point.Index(), &_in_PlaneMeshNormal);

        double ptNorm[3] = {_in_PlaneMeshNormal[0], _in_PlaneMeshNormal[1], _in_PlaneMeshNormal[2]};

        //Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(_in_PlaneMeshNormal[0], _SavedPlanePoints[p_id].nx);
        volcart::testing::SmallOrClose(_in_PlaneMeshNormal[1], _SavedPlanePoints[p_id].ny);
        volcart::testing::SmallOrClose(_in_PlaneMeshNormal[2], _SavedPlanePoints[p_id].nz);

        p_id++;

    }

    /// Cells ///
    // Initialize Cell Iterators
    ITKCellIterator _in_PlaneMeshCell = _in_PlaneMesh->GetCells()->Begin();

    int c = 0;

    while (_in_PlaneMeshCell != _in_PlaneMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        ITKPointInCellIterator _in_PlaneMeshPoint = _in_PlaneMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while (_in_PlaneMeshPoint != _in_PlaneMeshCell.Value()->PointIdsEnd()) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*_in_PlaneMeshPoint, _SavedPlaneCells[c].v1);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(*_in_PlaneMeshPoint, _SavedPlaneCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_in_PlaneMeshPoint, _SavedPlaneCells[c].v3);

            //increment points
            _in_PlaneMeshPoint++;
            counter++;

        }

        //increment cells
        ++_in_PlaneMeshCell;
        ++c;
    }
}
