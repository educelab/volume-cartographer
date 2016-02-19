//
// Created by Ryan Taber on 12/9/15.
//

#ifndef VC_PREBUILT_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE Metadata

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "shapes.h"
#include "vc_defines.h"
#include "io/plyWriter.h"


//TODO: update this stuff
/**************************************************************************************
 *                                                                                    *
 *  MetadataTest.cpp - tests the functionality of /v-c/common/datatypes/Metadata.cpp  *
 *  The ultimate goal of this file is the following:                                  *
 *                                                                                    *
 *        1. check whether a testing mesh, created by                                 *
 *           common/shapes/Plane.h, can be written into                               *
 *           a .ply file by common/io/plyWriter.cpp.                                  *
 *                                                                                    *
 *        2. read contents of ply file and compare data with testing mesh             *
 *                                                                                    *
 *  This file is broken up into a testing fixture, meshFix, which initializes the     *
 *  objects used in each of the two test cases.                                       *
 *                                                                                    *
 *  writeTest (test case):                                                            *
 *                                                                                    *
 *      attempts to write a testing mesh to file and compares final output path       *
 *      as a check for success. Note, this test always outputs the file as            *
 *      "output.ply" because there is no texture information included when writing.   *
 *                                                                                    *
 *  compareElements (test case):                                                      *
 *                                                                                    *
 *      Attempts to read in information from ply file using boost::path variable.     *
 *      As data is read in from the ply file, collections of points and faces are     *
 *      created to compare against points and faces created during initialization     *
 *      of the testing mesh by meshFix. The test then compares all non-commented      *
 *      data from the file against the testing mesh data to ensure equality.          *
 *                                                                                    *
 * Input:                                                                             *
 *     No required inputs for this sample test.                                       *
 *                                                                                    *
 * Test-Specific Output:                                                              *
 *     Specific test output only given on failure of any tests. Otherwise, general    *
 *     number of testing errors is output.                                            *
 *                                                                                    *
 * ************************************************************************************/



/*
 * This fixture builds objects for each of the test cases below that reference
 * the fixture as their second argument
 *
 */





//holder test
BOOST_AUTO_TEST_CASE(holder){

    BOOST_CHECK(true);
}