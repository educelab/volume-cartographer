#include <gtest/gtest.h>

#include <iostream>

#include "vc/core/io/UVMapIO.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/core/util/Logging.hpp"

static constexpr auto PI =
    double(3.141592653589793238462643383279502884198716939937510582097164L);

using namespace volcart;

/*
 * This fixture builds a uv map object for each of the test cases that will use
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
        for (auto& it : _Storage) {
            _BaseUVMap.set(pnt_id, it);
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

TEST(UVMapTest, Rotate45)
{
    // Construct UV map
    UVMap uvMap;
    uvMap.ratio(1., 1.);
    uvMap.set(0, {0, 0});
    uvMap.set(1, {1, 0});
    uvMap.set(2, {0, 1});
    uvMap.set(3, {1, 1});
    uvMap.set(4, {0.5, 0.5});

    auto uv45 = uvMap;
    UVMap::Rotate(uv45, 45. * PI / 180.);
    EXPECT_EQ(uv45.origin(), uvMap.origin());
    EXPECT_EQ(uv45.size(), uvMap.size());
    EXPECT_FLOAT_EQ(uv45.ratio().width, std::sqrt(2.));
    EXPECT_FLOAT_EQ(uv45.ratio().height, std::sqrt(2.));
    EXPECT_FLOAT_EQ(uv45.ratio().aspect, 1.);
    EXPECT_FLOAT_EQ(uv45.get(0)[0], 0.);
    EXPECT_FLOAT_EQ(uv45.get(0)[1], 0.5);
    EXPECT_FLOAT_EQ(uv45.get(1)[0], 0.5);
    EXPECT_FLOAT_EQ(uv45.get(1)[1], 0.);
    EXPECT_FLOAT_EQ(uv45.get(2)[0], 0.5);
    EXPECT_FLOAT_EQ(uv45.get(2)[1], 1.);
    EXPECT_FLOAT_EQ(uv45.get(3)[0], 1.);
    EXPECT_FLOAT_EQ(uv45.get(3)[1], 0.5);
    EXPECT_FLOAT_EQ(uv45.get(4)[0], 0.5);
    EXPECT_FLOAT_EQ(uv45.get(4)[1], 0.5);
}

TEST(UVMapTest, RotateMultiplesOf90)
{
    // Construct UV map
    UVMap uvMap;
    uvMap.ratio(10., 5.);
    uvMap.set(0, {0, 0});
    uvMap.set(1, {1, 0});
    uvMap.set(2, {0, 1});
    uvMap.set(3, {1, 1});
    uvMap.set(4, {0.5, 0.5});

    // Rotate 90
    auto uv90 = uvMap;
    UVMap::Rotate(uv90, UVMap::Rotation::CW90);
    EXPECT_EQ(uv90.origin(), uvMap.origin());
    EXPECT_EQ(uv90.size(), uvMap.size());
    EXPECT_EQ(uv90.ratio().width, uvMap.ratio().height);
    EXPECT_EQ(uv90.ratio().height, uvMap.ratio().width);
    EXPECT_EQ(uv90.ratio().aspect, 1. / uvMap.ratio().aspect);
    EXPECT_EQ(uv90.get(0), cv::Vec2d(1, 0));
    EXPECT_EQ(uv90.get(1), cv::Vec2d(1, 1));
    EXPECT_EQ(uv90.get(2), cv::Vec2d(0, 0));
    EXPECT_EQ(uv90.get(3), cv::Vec2d(0, 1));
    EXPECT_EQ(uv90.get(4), cv::Vec2d(0.5, 0.5));

    // Rotate 180
    auto uv180 = uvMap;
    UVMap::Rotate(uv180, UVMap::Rotation::CW180);
    EXPECT_EQ(uv180.origin(), uvMap.origin());
    EXPECT_EQ(uv180.size(), uvMap.size());
    EXPECT_EQ(uv180.ratio().width, uvMap.ratio().width);
    EXPECT_EQ(uv180.ratio().height, uvMap.ratio().height);
    EXPECT_EQ(uv180.ratio().aspect, uvMap.ratio().aspect);
    EXPECT_EQ(uv180.get(0), cv::Vec2d(1, 1));
    EXPECT_EQ(uv180.get(1), cv::Vec2d(0, 1));
    EXPECT_EQ(uv180.get(2), cv::Vec2d(1, 0));
    EXPECT_EQ(uv180.get(3), cv::Vec2d(0, 0));
    EXPECT_EQ(uv180.get(4), cv::Vec2d(0.5, 0.5));

    // Rotate 270 (90 CCW)
    auto uv270 = uvMap;
    UVMap::Rotate(uv270, UVMap::Rotation::CCW90);
    EXPECT_EQ(uv270.origin(), uvMap.origin());
    EXPECT_EQ(uv270.size(), uvMap.size());
    EXPECT_EQ(uv270.ratio().width, uvMap.ratio().height);
    EXPECT_EQ(uv270.ratio().height, uvMap.ratio().width);
    EXPECT_EQ(uv270.ratio().aspect, 1. / uvMap.ratio().aspect);
    EXPECT_EQ(uv270.get(0), cv::Vec2d(0, 1));
    EXPECT_EQ(uv270.get(1), cv::Vec2d(0, 0));
    EXPECT_EQ(uv270.get(2), cv::Vec2d(1, 1));
    EXPECT_EQ(uv270.get(3), cv::Vec2d(1, 0));
    EXPECT_EQ(uv270.get(4), cv::Vec2d(0.5, 0.5));
}