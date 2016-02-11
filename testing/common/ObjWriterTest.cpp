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

        std::cout << "Writing mesh..." << std::endl;

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

    //store mesh data created by fixture
    std::vector<VC_Vertex> testPoints = mesh.getPoints();
    std::vector<VC_Cell> testCells = mesh.getCells();

    //   Declare vectors to hold mesh points and cells from saved obj file
    std::vector<VC_Vertex> savedPoints;
    std::vector<VC_Cell> savedCells;

    volcart::testing::ParsingHelpers::parseObjFile("output.obj", savedPoints, savedCells);


    // Now to test the objPoints created during fixture init vs points read in from file.
    for (size_t p = 0; p < testPoints.size(); p++){

        BOOST_TEST_MESSAGE("Checking Point " << p);

        //checking the x,y,z,nx,ny,nz components

        BOOST_CHECK_EQUAL(testPoints[p].x, savedPoints[p].x);
        BOOST_CHECK_EQUAL(testPoints[p].y, savedPoints[p].y);
        BOOST_CHECK_EQUAL(testPoints[p].z, savedPoints[p].z);

        BOOST_CHECK_EQUAL(testPoints[p].nx, savedPoints[p].nx);
        BOOST_CHECK_EQUAL(testPoints[p].ny, savedPoints[p].ny);
        BOOST_CHECK_EQUAL(testPoints[p].nz, savedPoints[p].nz);

    }

    // Now to test the objPoints created during fixture init vs points read in from file.
    for (size_t c = 0; c < testCells.size(); c++){

        BOOST_TEST_MESSAGE("Checking Cell: " << c);

        //checking the v1,v2,v3 components
        //note: parseObjFile() accounts for the 'one off' of the vertex values in the .obj file

        BOOST_CHECK_EQUAL(testCells[c].v1, savedCells[c].v1);
        BOOST_CHECK_EQUAL(testCells[c].v2, savedCells[c].v2);
        BOOST_CHECK_EQUAL(testCells[c].v3, savedCells[c].v3);

    }

}

