//
// Created by Seth Parker on 11/22/16.
//

#define BOOST_TEST_MODULE PerPixelMap

#include <boost/test/unit_test.hpp>

#include "core/types/PerPixelMap.hpp"

using namespace volcart;

BOOST_AUTO_TEST_CASE(GetSetIO)
{
    // Make the PPM
    volcart::PerPixelMap ppm(10, 10);
    for (auto y = 0; y < 10; ++y) {
        for (auto x = 0; x < 10; ++x) {
            ppm(y, x) = {x, y, (x + y) / 2.0, x, y, (x + y) / 2.0};
        }
    }
    PerPixelMap::WritePPM("PPM_Test.ppm", ppm);

    // Get the PPM
    auto result = PerPixelMap::ReadPPM("PPM_Test.ppm");
    for (auto y = 0; y < 10; ++y) {
        for (auto x = 0; x < 10; ++x) {
            BOOST_CHECK_EQUAL(result(y, x), ppm(y, x));
        }
    }
}