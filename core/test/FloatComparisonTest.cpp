//
// Created by Seth on 11/5/16.
//

#define BOOST_TEST_MODULE FloatComparison

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "core/util/FloatComparison.hpp"

using namespace volcart;

struct SmallDifferences {
    SmallDifferences()
    {
        for (auto i = 0; i < 10; ++i) {
            f1 += float(0.1);
            d1 += float(0.1);
        }
    }

    float f0 = 1.0;
    float f1 = 0.0;
    double d0 = 1.0;
    double d1 = 0.0;
};

BOOST_FIXTURE_TEST_CASE(AlmostEqualTest, SmallDifferences)
{
    BOOST_CHECK(f0 != f1);
    BOOST_CHECK(AlmostEqual(f0, f1));

    BOOST_CHECK(d0 != d1);
    BOOST_CHECK(AlmostEqual(d0, d1));
}
