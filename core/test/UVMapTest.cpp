#include <gtest/gtest.h>

#include <iostream>

#include "vc/core/io/UVMapIO.hpp"
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

TEST(UVMapTest, WriteAndRead)
{
    // Construct a UVMap
    UVMap uvMap;

    // Fill it randomly
    cv::RNG rng(std::time(nullptr));
    uvMap.setOrigin(static_cast<UVMap::Origin>(rng.uniform(0, 4)));
    auto size = static_cast<std::size_t>(rng.uniform(100, 1000));
    for (size_t i = 0; i < size; i++) {
        size_t id = rng.uniform(0, size);
        auto u = rng.uniform(0.0, 1.0);
        auto v = rng.uniform(0.0, 1.0);
        uvMap.set(id, {u, v});
    }

    // Write the UVMap
    io::WriteUVMap("WriteUVMap.uvm", uvMap);

    // Read the UVMap
    auto uvMapClone = io::ReadUVMap("WriteUVMap.uvm");

    // Check that the major parts match
    EXPECT_EQ(uvMapClone.origin(), uvMap.origin());
    EXPECT_EQ(uvMapClone.size(), uvMap.size());
    EXPECT_EQ(uvMapClone.ratio().width, uvMap.ratio().width);
    EXPECT_EQ(uvMapClone.ratio().height, uvMap.ratio().height);
    EXPECT_EQ(uvMapClone.ratio().aspect, uvMap.ratio().aspect);

    for (const auto& m : uvMapClone.as_map()) {
        const auto& id = m.first;
        const auto& uv = m.second;
        EXPECT_EQ(uv, uvMap.get(id));
    }
}