#include <gtest/gtest.h>

#include <iostream>

#include "vc/core/types/UVMap.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;

/*
 * This fixture builds  a uv map object for each of the test cases that will use
 * the
 */

class CreateUVMapFixture : public ::testing::Test
{
public:
    CreateUVMapFixture()
    {
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

    // Init uvMap
    volcart::UVMap _BaseUVMap;
    std::vector<cv::Vec2d> _Storage;
};

// Check that we can set, get, and re-set values for a point
TEST(UVMapTest, GetSetTest)
{
    volcart::UVMap map;
    cv::Vec2d p{0.0, 0.0};
    map.set(0, p);
    EXPECT_EQ(map.get(0), p);

    p[0] = 1.0;
    map.set(0, p);
    EXPECT_EQ(map.get(0), p);
}

// Check the fun origin transformation part of this class
TEST_F(CreateUVMapFixture, TransformationTest)
{

    // get the original points
    volcart::UVMap map = _BaseUVMap;
    map.setOrigin(UVMap::Origin::TopLeft);  // standard origin

    // Retrieve mappings relative to the top-left (0,0)
    _BaseUVMap.setOrigin(UVMap::Origin::TopLeft);
    int pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0]);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1]);  // v

        EXPECT_EQ(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        EXPECT_EQ(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }

    Logger()->debug(
        "Transforming against (0,1) and comparing expected results");

    // Retrieve mappings relative to the bottom-left (0,1)
    _BaseUVMap.setOrigin(UVMap::Origin::BottomLeft);
    pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0] - 0);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1] - 1);  // v

        EXPECT_EQ(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        EXPECT_EQ(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }

    Logger()->debug(
        "Transforming against (1,0) and comparing expected results");

    // Retrieve mappings relative to the top-right (1,0)
    _BaseUVMap.setOrigin(UVMap::Origin::TopRight);
    pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0] - 1);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1] - 0);  // v

        EXPECT_EQ(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        EXPECT_EQ(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }

    // Retrieve mappings relative to the bottom-right (1,1)
    _BaseUVMap.setOrigin(UVMap::Origin::BottomRight);
    pnt_id = 0;
    for (auto it = _Storage.begin(); it != _Storage.end(); ++it) {

        // set expected values
        cv::Vec2d ExpectedValues;
        ExpectedValues[0] = std::abs(map.get(pnt_id)[0] - 1);  // u
        ExpectedValues[1] = std::abs(map.get(pnt_id)[1] - 1);  // v

        EXPECT_EQ(ExpectedValues[0], _BaseUVMap.get(pnt_id)[0]);
        EXPECT_EQ(ExpectedValues[1], _BaseUVMap.get(pnt_id)[1]);

        ++pnt_id;
    }
}