//
// Created by Ryan Taber on 12/9/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE plyWriter

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "shapes.h"
#include "vc_defines.h"
#include "io/plyWriter.h"
#include "testing/parsingHelpers.h"


/************************************************************************************
 *                                                                                  *
 *  plyWriterTest.cpp - tests the functionality of /v-c/common/io/plyWriter.cpp     *
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
 * ************************************************************************************/



/*
 * This fixture builds objects for each of the test cases below that reference
 * the fixture as their second argument
 *
 */

struct meshFix {

  meshFix() {

      //create the mesh for all the test cases to use
      _mesh = mesh.itkMesh();

      BOOST_TEST_MESSAGE("setting up mesh");
  }

  ~meshFix(){ BOOST_TEST_MESSAGE("cleaning up meshFix"); }

  //file path and objwriter to be used in cases
  volcart::io::objWriter mesh_writer;
  boost::filesystem::path objPath;
  VC_MeshType::Pointer _mesh ;
  volcart::shapes::Plane mesh;
};


// Test for checking successful write
BOOST_FIXTURE_TEST_CASE(writeTest, meshFix) {

  std::cout << "Writing object" << std::endl;

  mesh_writer.setPath("nothing");
  //mesh_writer.setUVMap( uvMap );
  // mesh_writer.setTexture( uvImg );

  objPath = mesh_writer.getPath();
  objPath = boost::filesystem::absolute(objPath);


  // mesh_writer.write() runs validate() as well, but this lets us handle a mesh that can't be validated.
  if (mesh_writer.validate())
      mesh_writer.write();
  else {
      mesh_writer.setPath("output.obj");
      mesh_writer.setMesh(_mesh);
      mesh_writer.write();
}

//check the file path from the mesh_writer.write() call above
//compare() returns 0 only if paths are same value lexicographically-speaking
//checking "output.obj" here because the mesh_writer shouldn't validate in the current case
BOOST_CHECK_EQUAL(mesh_writer.getPath().compare("output.obj"), 0);

}

BOOST_FIXTURE_TEST_CASE(compareElements, meshFix){


}