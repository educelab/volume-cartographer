//
// Created by Seth on 11/5/16.
//

#define BOOST_TEST_MODULE FloatComparison

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "common/util/FloatComparison.h"

using namespace volcart;

struct SmallFloats {
    float f0 = std::numeric_limits<float>::min();
    float f1 = 1e-8;

    double d0 = std::numeric_limits<double>::min();
    double d1 = 1e-24;
};

BOOST_FIXTURE_TEST_CASE(AlmostEqualTest, SmallFloats)
{
    BOOST_CHECK_EQUAL(true, AlmostEqual(f0, f1));
    BOOST_CHECK_EQUAL(true, AlmostEqual(d0, d1));
}