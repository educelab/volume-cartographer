//
// Created by Ryan Taber on 12/11/15.
//

#define BOOST_TEST_MODULE uvMap

#include <boost/test/unit_test.hpp>
#include "vc/core/types/UVMap.hpp"
#include "vc/core/vc_defines.hpp"

using namespace volcart;

/***************************************************************************************
 *                                                                                     *
 *  UVMapTest.cpp - tests the functionality of /v-c/core/UVMap.h *
 *  The ultimate goal of this file is the following: *
 *                                                                                     *
 *        1. check whether transformations of uv map coords relative to the four
 * *
 *           possible origin locations (see vc_defines for naming) *
 *                                                                                     *
 *  This file is broken up into a test fixture, uvFix, which initializes the
 * objects   *
 *  used in each of the four test cases. *
 *                                                                                     *
 *        1. transformationTest - looks at four origin options *
 *           -relative to top left (0,0) *
 *           -relative to bottom left (0,1) *
 *           -relative to top right (1,0) *
 *           -relative to bottom right (1,1) *
 *                                                                                     *
 * Input: *
 *     No required inputs for this sample test. Note: the output.ply must be
 * copied    *
 *     from test_data/common to curr_bin_dir when building, which is handled by
 * cmake. *
 *                                                                                     *
 * Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general     *
 *     number of testing errors is output. *
 *                                                                                     *
 * *************************************************************************************/

/*
 * This fixture builds  a uv map object for each of the test cases that will use
 * the
 */

struct CreateUVMapFixture {

    CreateUVMapFixture()
    {

        std::cout << "Constructing uv map..." << std::endl;

        // fill storage vector
        for (double u = 0; u <= 1; u += 0.25) {
            for (double v = 0; v <= 1; v += 0.25) {
                cv::Vec2d uv(u, v);
                _Storage.push_back(uv);
            }
        }

        // Insert mappings relative to the top-left (default origin)
        int pnt_id = 0;
        for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {
            _BaseUVMap.set(pnt_id, *it);
            ++pnt_id;
        }
    }

    ~CreateUVMapFixture() { std::cout << "Destroying uv map..." << std::endl; }

    // Init uvMap
    volcart::UVMap _BaseUVMap;
    std::vector<cv::Vec2d> _Storage;
};

// Check that we can set, get, and re-set values for a point
BOOST_AUTO_TEST_CASE(GetSetTest)
{
    volcart::UVMap map;
    cv::Vec2d p{0.0, 0.0};
    map.set(0, p);
    BOOST_CHECK_EQUAL(map.get(0), p);

    p[0] = 1.0;
    map.set(0, p);
    BOOST_CHECK_EQUAL(map.get(0), p);
}

// Check the fun origin transformation part of this class
BOOST_FIXTURE_TEST_CASE(TransformationTest, CreateUVMapFixture)
{

    // get the original points
    volcart::UVMap map = _BaseUVMap;
    map.origin(VC_ORIGIN_TOP_LEFT);  // standard origin

    std::cout << "Transforming against (0,0) and comparing expected results"
              << std::endl;

    // Retrieve mappings relative to the top-left (0,0)
    _BaseUVMap.origin(VC_ORIGIN_TOP_LEFT);
    int pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0]);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1]);  // v

        BOOST_CHECK_EQUAL(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        BOOST_CHECK_EQUAL(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }

    std::cout << "Transforming against (0,1) and comparing expected results"
              << std::endl;

    // Retrieve mappings relative to the bottom-left (0,1)
    _BaseUVMap.origin(VC_ORIGIN_BOTTOM_LEFT);
    pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0] - 0);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1] - 1);  // v

        BOOST_CHECK_EQUAL(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        BOOST_CHECK_EQUAL(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }

    std::cout << "Transforming against (1,0) and comparing expected results"
              << std::endl;

    // Retrieve mappings relative to the top-right (1,0)
    _BaseUVMap.origin(VC_ORIGIN_TOP_RIGHT);
    pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0] - 1);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1] - 0);  // v

        BOOST_CHECK_EQUAL(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        BOOST_CHECK_EQUAL(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }

    std::cout << "Transforming against (1,1) and comparing expected results"
              << std::endl;

    // Retrieve mappings relative to the bottom-right (1,1)
    _BaseUVMap.origin(VC_ORIGIN_BOTTOM_RIGHT);
    pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0] - 1);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1] - 1);  // v

        BOOST_CHECK_EQUAL(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        BOOST_CHECK_EQUAL(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }
}
