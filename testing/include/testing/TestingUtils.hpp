// Testing Utilities
// Created by Seth Parker - 2016/07/27
#pragma once

#include <boost/test/unit_test.hpp>

namespace volcart
{
namespace testing
{

// Uses Boost.Test to check whether double values are close enough
// If the abs expected value is smaller than smallTolerance, check if the
// observed value is as well. Otherwise, check whether the relative diff is
// within percentDiffTolerance (in percentage units).
// Taken from: http://stackoverflow.com/a/20050381/1917043
inline void SmallOrClose(
    double observed,
    double expected,
    double smallTolerance = 0.00001,
    double percentDiffTolerance = 1.0)
{
    if (std::fabs(expected) < smallTolerance) {
        BOOST_CHECK_SMALL(observed, smallTolerance);
    } else {
        BOOST_CHECK_CLOSE(observed, expected, percentDiffTolerance);
    }
}  // SmallOrClose

}  // testing
}  // volcart
