//
// Created by Ryan Taber on 12/9/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE plyWriter

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <vc_defines.h>
#include "shapes.h"
#include "io/plyWriter.h"
#include "parsingHelpers.h"


/************************************************************************************
 *                                                                                  *
 *  plyWriterTest.cpp - tests the functionality of /v-c/common/plyWriter.cpp        *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether a testing mesh, created by                               *
 *           common/shapes/Plane.h, can be written into                             *
 *           a .ply file by common/io/plyWriter.cpp.                                *
 *                                                                                  *
 *        2. read contents of ply file and compare data with testing mesh           *
 *                                                                                  *
 *  This file is broken up into a testing fixture, meshFix, which initializes the   *
 *  objects used in each of the two test cases.                                     *
 *                                                                                  *
 *  writeTest (test case):                                                          *
 *                                                                                  *
 *      attempts to write a testing mesh to file and compares final output path     *
 *      as a check for success. Note, this test always outputs the file as          *
 *      "output.ply" because there is no texture information included when writing. *
 *                                                                                  *
 *  compareElements (test case):                                                    *
 *                                                                                  *
 *      Attempts to read in information from ply file using boost::path variable.   *
 *      As data is read in from the ply file, collections of points and faces are   *
 *      created to compare against points and faces created during initialization   *
 *      of the testing mesh by meshFix. The test then compares all non-commented    *
 *      data from the file against the testing mesh data to ensure equality.        *
 *                                                                                  *
 * Input:                                                                           *
 *     No required inputs for this sample test.                                     *
 *                                                                                  *
 * Test-Specific Output:                                                            *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output.                                          *
 *                                                                                  *
 * **********************************************************************************/



/*
 * This fixture builds objects for each of the test cases below that reference
 * the fixture as their second argument
 *
 */

struct meshFix {

  meshFix() {

      //create the mesh for all the test cases to use
      _mesh = mesh.itkMesh();

      BOOST_TEST_MESSAGE("setting up mesh...");
  }

  ~meshFix(){ BOOST_TEST_MESSAGE("cleaning up meshFix..."); }

  //file path and plyWriter to be used in cases
  volcart::io::plyWriter mesh_writer;
  boost::filesystem::path objPath;
  VC_MeshType::Pointer _mesh ;
  volcart::shapes::Plane mesh;

};


// Test for checking successful write
BOOST_FIXTURE_TEST_CASE(writeTest, meshFix) {

  std::cout << "Writing mesh to ply file..." << std::endl;

  mesh_writer.setPath("nothing");
  //mesh_writer.setUVMap( uvMap );
  // mesh_writer.setTexture( uvImg );

  objPath = mesh_writer.getPath();
  objPath = boost::filesystem::absolute(objPath);


  // mesh_writer.write() runs validate() as well, but this lets us handle a mesh that can't be validated.
  if (mesh_writer.validate())
      mesh_writer.write();
  else {
      mesh_writer.setPath("output.ply");
      mesh_writer.setMesh(_mesh);
      mesh_writer.write();
  }

  //check the file path from the mesh_writer.write() call above
  //compare() returns 0 only if paths are same value lexicographically-speaking
  //checking "output.ply" here because the mesh_writer shouldn't validate in the current case
  BOOST_CHECK_EQUAL(mesh_writer.getPath().compare("output.ply"), 0);

}

BOOST_FIXTURE_TEST_CASE(compareElements, meshFix){


    //init vectors to hold vertices and faces saved in output.ply
    std::vector<VC_Vertex> savedPoints, fixturePoints;
    std::vector<VC_Cell> savedCells, fixtureCells;

    //read in data from saved output.ply via parsingHelpers::parsePlyFile
    volcart::testing::ParsingHelpers::parsePlyFile("output.ply", savedPoints, savedCells);

    /* Now the data from the .ply file is in the point and cell vectors, and we're
     * ready to compare with data from the _mesh object created by meshFix
     *
     * Note: plyWriter is only writing x/y/z/nx/ny/nz point data, so that is
     *       all we're comparing for equivalency below
     */


    /// Points ///
    std::cerr << "Comparing points..." << std::endl;
    for (int p = 0; p < savedPoints.size(); p++){

        BOOST_CHECK_EQUAL(savedPoints[p].x, _mesh->GetPoint(p)[0]);
        BOOST_CHECK_EQUAL(savedPoints[p].y, _mesh->GetPoint(p)[1]);
        BOOST_CHECK_EQUAL(savedPoints[p].z, _mesh->GetPoint(p)[2]);
    }

    // Normals //
    std::cerr << "Comparing normals..." << std::endl;
    int p_id = 0;
    for ( VC_PointsInMeshIterator point = _mesh->GetPoints()->Begin(); point != _mesh->GetPoints()->End(); ++point ) {

        VC_PixelType _meshNormal;
        _mesh->GetPointData(point.Index(), &_meshNormal);

        double ptNorm[3] = {_meshNormal[0], _meshNormal[1], _meshNormal[2]};

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(_meshNormal[0], savedPoints[p_id].nx);
        BOOST_CHECK_EQUAL(_meshNormal[1], savedPoints[p_id].ny);
        BOOST_CHECK_EQUAL(_meshNormal[2], savedPoints[p_id].nz);

        p_id++;

    }

    /// Cells ///
    std::cerr << "Comparing cells..." << std::endl;

    //compare number of cells in each mesh
    BOOST_CHECK_EQUAL(_mesh->GetNumberOfCells(), savedCells.size());

    // Initialize Cell Iterators
    VC_CellIterator _meshCell = _mesh->GetCells()->Begin();

    int c = 0;

    while (_meshCell != _mesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator _meshPoint = _meshCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while (_meshPoint != _meshCell.Value()->PointIdsEnd()) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*_meshPoint, savedCells[c].v1);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(*_meshPoint, savedCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_meshPoint, savedCells[c].v3);

            //increment points
            _meshPoint++;
            counter++;

        }

        //increment cells
        ++_meshCell;
        ++c;
    }
}