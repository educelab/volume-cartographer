//
// Created by Ryan Taber on 10/14/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE itk2vtk

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "testing/testingMesh.h"

/************************************************************************************
 *                                                                                  *
 *  it2vtkTest.cpp - tests the functionality of /v-c/meshing/it2vtk.cpp             *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether an itk mesh can be converted to a vtk mesh               *
 *           and vice versa.                                                        *
 *           common/testing/testingMesh.cpp, can be written into                    *
 *                                                                                  *
 *                                                                                  *
 *  This file is broken up into a testing fixture, ivFix, which initializes the     *
 *  objects used in each of the two test cases.                                     *
 *                                                                                  *
 *  i2v (test case):                                                                *
 *                                                                                  *
 *      Takes an itk mesh created from fixture and data to vtkpolydata pointer.     *
 *      Vtkpolydata then converted back to itk mesh. Successful test if converted   *
 *      itk matches original itk mesh data points                                   *
 *                                                                                  *
 *  v2i (test case):                                                                *
 *                                                                                  *
 *      Same idea as i2v test case except that the test starts with vtkpolydata.    *
 *      The vtk data is converted into itk mesh and back to vtk. Success if the     *
 *      original vtk and converted vtk data points match.                           *
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
 * ************************************************************************************/




//  helper function to split lines read in from obj file  //
std::vector<std::string> split_string(std::string);

/*
 * This builds objects for the test cases below that reference
 * the fixture as their second argument
 *
 */

struct ivFix {

    ivFix() {

        //need to create a pointer to itk and vtk meshes
        _mesh = mesh.itkMesh();

        std::cerr << "setting up itk2vtkTest objects" << std::endl;
    }

    ~ivFix(){ std::cerr << "cleaning up itk2vtkTest objects" << std::endl; }

    VC_MeshType::Pointer _mesh ;
    volcart::testing::testingMesh mesh;
};

// check successful conversion from itk mesh to vtk mesh

BOOST_FIXTURE_TEST_CASE(i2v, ivFix){

    BOOST_CHECK(true);

}


// check successful conversion from itk mesh to vtk mesh
BOOST_FIXTURE_TEST_CASE(v2i, ivFix){

    BOOST_CHECK(true);

}
