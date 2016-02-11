//
// Created by Ryan Taber on 12/11/15.
//

#ifndef VC_PREBUILT_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE uvMap

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "vc_datatypes.h"


/***************************************************************************************
 *                                                                                     *
 *  UVMapTest.cpp - tests the functionality of /v-c/common/UVMap.h                     *
 *  The ultimate goal of this file is the following:                                   *
 *                                                                                     *
 *        1. check whether transformations of uv map coords relative to the four       *
 *           possible origin locations (see vc_defines for naming)                     *
 *                                                                                     *
 *  This file is broken up into a test fixture, uvFix, which initializes the objects   *
 *  used in each of the four test cases.                                               *
 *                                                                                     *
 *        1. transformationTest - looks at four origin options                         *
 *           -relative to top left (0,0)                                               *
 *           -relative to bottom left (0,1)                                            *
 *           -relative to top right (1,0)                                              *
 *           -relative to bottom right (1,1)                                           *
 *                                                                                     *
 * Input:                                                                              *
 *     No required inputs for this sample test. Note: the output.ply must be copied    *
 *     from test_data/common to curr_bin_dir when building, which is handled by cmake. *
 *                                                                                     *
 * Test-Specific Output:                                                               *
 *     Specific test output only given on failure of any tests. Otherwise, general     *
 *     number of testing errors is output.                                             *
 *                                                                                     *
 * *************************************************************************************/



/*
 * This fixture builds  a uv map object for each of the test cases that will use the
 *
 */

 struct uvFix{

     uvFix(){

         std::cout << "Constructing uv map..." << std::endl;

         //fill storage vector
         for (double u = 0; u <= 1; u += 0.25 ) {
             for (double v = 0; v <= 1; v += 0.25) {
                 cv::Vec2d uv( u, v );
                 storage.push_back(uv);
             }
         }

         // Insert mappings relative to the top-left (default origin)
         int pointID = 0;
         for ( auto it = storage.begin(); it != storage.end(); ++it ) {
             baseUVMap.set( pointID, *it );
             ++pointID;
         }
     }


     ~uvFix() {
         std::cout << "Destroying uv map..." << std::endl;
     }

     //Init uvMap
     volcart::UVMap baseUVMap;
     std::vector<cv::Vec2d> storage;


 };


BOOST_FIXTURE_TEST_CASE(transformationTest, uvFix){

    //get the original points
    volcart::UVMap map = baseUVMap;
    map.origin(VC_ORIGIN_TOP_LEFT); //standard origin

    std::cout << "Transforming against (0,0) and comparing expected results" << std::endl;
    // Retrieve mappings relative to the top-left (0,0)
    baseUVMap.origin(VC_ORIGIN_TOP_LEFT);
    int pointID = 0;
    for ( auto it = storage.begin(); it != storage.end(); ++it ) {

        //just printing to see what we're looking at here
        //std::cout << "Trans Point: " << pointID << " | " << baseUVMap.get(pointID) << std::endl;

        //set expected values
        cv::Vec2d expected;
        expected[0] = std::abs(map.get(pointID)[0] - 0);  //u
        expected[1] = std::abs(map.get(pointID)[1] - 0);  //v


        BOOST_CHECK_EQUAL(expected[0], baseUVMap.get(pointID)[0]);
        BOOST_CHECK_EQUAL(expected[1], baseUVMap.get(pointID)[1]);

        ++pointID;
  }



    std::cout << "Transforming against (0,1) and comparing expected results" << std::endl;

    // Retrieve mappings relative to the bottom-left (0,1)
    baseUVMap.origin(VC_ORIGIN_BOTTOM_LEFT);
    pointID = 0;
    for ( auto it = storage.begin(); it != storage.end(); ++it ) {

        //std::cout << "Trans Point: " << pointID << " | " << baseUVMap.get(pointID) << std::endl;

        //set expected values
        cv::Vec2d expected;
        expected[0] = std::abs(map.get(pointID)[0] - 0);  //u
        expected[1] = std::abs(map.get(pointID)[1] - 1);  //v


        BOOST_CHECK_EQUAL(expected[0], baseUVMap.get(pointID)[0]);
        BOOST_CHECK_EQUAL(expected[1], baseUVMap.get(pointID)[1]);

        ++pointID;
    }


    std::cout << "Transforming against (1,0) and comparing expected results" << std::endl;

    // Retrieve mappings relative to the top-right (1,0)
    baseUVMap.origin(VC_ORIGIN_TOP_RIGHT);
    pointID = 0;
    for ( auto it = storage.begin(); it != storage.end(); ++it ) {

        //std::cout << "Trans Point: " << pointID << " | " << baseUVMap.get(pointID) << std::endl;

        //set expected values
        cv::Vec2d expected;
        expected[0] = std::abs(map.get(pointID)[0] - 1);  //u
        expected[1] = std::abs(map.get(pointID)[1] - 0);  //v


        BOOST_CHECK_EQUAL(expected[0], baseUVMap.get(pointID)[0]);
        BOOST_CHECK_EQUAL(expected[1], baseUVMap.get(pointID)[1]);

        ++pointID;
    }


    std::cout << "Transforming against (1,1) and comparing expected results" << std::endl;

    // Retrieve mappings relative to the bottom-right (1,1)
    baseUVMap.origin(VC_ORIGIN_BOTTOM_RIGHT);
    pointID = 0;
    for ( auto it = storage.begin(); it != storage.end(); ++it ) {

        //std::cout << "Trans Point: " << pointID << " | " << baseUVMap.get(pointID) << std::endl;

        //set expected values
        cv::Vec2d expected;
        expected[0] = std::abs(map.get(pointID)[0] - 1);  //u
        expected[1] = std::abs(map.get(pointID)[1] - 1);  //v


        BOOST_CHECK_EQUAL(expected[0], baseUVMap.get(pointID)[0]);
        BOOST_CHECK_EQUAL(expected[1], baseUVMap.get(pointID)[1]);

        ++pointID;
    }


}