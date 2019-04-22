#include <iostream>
#include <gtest/gtest.h>

#include "vc/core/util/FloatComparison.hpp"

using namespace volcart;

class FloatComparison : public ::testing::Test
{
public:
    FloatComparison()
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

TEST_F(FloatComparison, AlmostEqual)
{
    EXPECT_NE(f0, f1);
    EXPECT_TRUE(AlmostEqual(f0, f1));
    EXPECT_NE(d0, d1);
    EXPECT_TRUE(AlmostEqual(d0, d1));
}