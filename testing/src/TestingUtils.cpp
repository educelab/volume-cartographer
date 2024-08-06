#include "vc/testing/TestingUtils.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace vctest = volcart::testing;

void vctest::SmallOrClose(
    double observed,
    double expected,
    double smallTolerance,
    double pctDiffTolerance)
{
    // check to see that observed is less than or almost equal to smallTolerance
    if (std::fabs(expected) < smallTolerance) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, observed, smallTolerance);
    } else {
        EXPECT_NEAR(
            observed, expected, std::fabs(expected * (pctDiffTolerance / 100)));
    }
}

void vctest::ExpectNear(
    double observed, double expected, double pctDiffTolerance)
{
    double absError =
        std::fabs((observed + expected) / 2 + pctDiffTolerance / 100);
    EXPECT_NEAR(observed, expected, absError);
}

void vctest::AssertNear(
    double observed, double expected, double pctDiffTolerance)
{
    double absError =
        std::fabs((observed + expected) / 2 + pctDiffTolerance / 100);
    ASSERT_NEAR(observed, expected, absError);
}