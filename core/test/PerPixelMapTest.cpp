#include <gtest/gtest.h>

#include "vc/core/types/PerPixelMap.hpp"

using namespace volcart;

TEST(PerPixelMap, WriteRead)
{
    // Build a PPM
    PerPixelMap ppm(10, 10);
    for (auto y = 0; y < 10; ++y) {
        for (auto x = 0; x < 10; ++x) {
            auto dx = static_cast<double>(x);
            auto dy = static_cast<double>(y);
            ppm(y, x) = {dx, dy, (dx + dy) / 2.0, dx, dy, (dx + dy) / 2.0};
        }
    }

    // Write the PPM
    std::string path{"vc_core_PerPixelMap_WriteRead.ppm"};
    EXPECT_NO_THROW(PerPixelMap::WritePPM(path, ppm));

    // Read the PPM
    PerPixelMap result;
    EXPECT_NO_THROW(result = PerPixelMap::ReadPPM(path));

    // Test the values
    for (auto y = 0; y < 10; ++y) {
        for (auto x = 0; x < 10; ++x) {
            EXPECT_EQ(result(y, x), ppm(y, x));
        }
    }
}