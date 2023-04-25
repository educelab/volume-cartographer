#include <gtest/gtest.h>

#include <iostream>
#include <numeric>

#include <opencv2/core.hpp>

#include "vc/segmentation/lrps/Derivative.hpp"
#include "vc/testing/TestingUtils.hpp"

using Voxel = cv::Vec3d;
using namespace volcart::segmentation;

static const double percentTol = 0.01;  // %
static const double smallTol = 1e-6;

class ConstantDoubleVectorFixture : public ::testing::Test
{
public:
    std::vector<double> _v;

    ConstantDoubleVectorFixture() : _v(20)
    {
        std::fill(std::begin(_v), std::end(_v), 1.0);
    }
};

class YEqualsXSquaredDoubleVectorFixture : public ::testing::Test
{
public:
    std::vector<double> _xs;
    std::vector<double> _ys;

    YEqualsXSquaredDoubleVectorFixture() : _xs(20)
    {
        int start = 2;
        std::iota(std::begin(_xs), std::end(_xs), start);
        for (auto x : _xs) {
            _ys.push_back(x * x);
        }
    }
};

////////////////////////////////////////////////////////////////////////////////
// constant D1 testing
// Note: Using BOOST_CHECK_SMALL since _v[i] result is 0. If we use
// CHECK_CLOSE, it will fail because 0 * percent = 0

TEST_F(ConstantDoubleVectorFixture, ConstantD1Forward)
{
    for (size_t i = 0; i < _v.size() - 1; ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D1Forward(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD1Backward)
{
    for (size_t i = 1; i < _v.size(); ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D1Backward(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD1Central)
{
    for (size_t i = 1; i < _v.size() - 1; ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D1Central(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD1FivePointStencil)
{
    for (size_t i = 2; i < _v.size() - 2; ++i) {
        EXPECT_PRED_FORMAT2(
            ::testing::DoubleLE, D1FivePointStencil(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD1At)
{
    for (size_t i = 0; i < _v.size(); ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D1At(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD1)
{
    for (auto d : D1(_v)) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, d, smallTol);
    }
}

////////////////////////////////////////////////////////////////////////////////
// constant D2 testing

TEST_F(ConstantDoubleVectorFixture, ConstantD2Forward)
{
    for (size_t i = 0; i < _v.size() - 2; ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D2Forward(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD2Backward)
{
    for (size_t i = 2; i < _v.size(); ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D2Backward(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD2Central)
{
    for (size_t i = 1; i < _v.size() - 1; ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D2Central(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD2FivePointStencil)
{
    for (size_t i = 2; i < _v.size() - 2; ++i) {
        EXPECT_PRED_FORMAT2(
            ::testing::DoubleLE, D2FivePointStencil(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD2At)
{
    for (size_t i = 0; i < _v.size(); ++i) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, D2At(_v, i), smallTol);
    }
}

TEST_F(ConstantDoubleVectorFixture, ConstantD2)
{
    for (auto d : D2(_v)) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, d, smallTol);
    }
}

////////////////////////////////////////////////////////////////////////////////
// y = x^2 D1 testing

// Forward difference error is given by:
//     error = - (h / 2) * f''(c)
// for some c in [x, x+h]. For y = x^2, f''(x) = 2, so error is -1
TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD1Forward)
{
    for (size_t i = 0; i < _ys.size() - 1; ++i) {
        volcart::testing::ExpectNear(
            D1Forward(_ys, i), 2 * _xs[i] + 1, percentTol);
    }
}

// Backward difference error is given by:
//     error = (h / 2) * f''(c)
// for some c in [x, x+h]. For y = x^2, f''(x) = 2, so error is +1
TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD1Backward)
{
    for (size_t i = 1; i < _ys.size(); ++i) {
        volcart::testing::ExpectNear(
            D1Backward(_ys, i), 2 * _xs[i] - 1, percentTol);
    }
}

// Central difference error is given by:
//     error = (h^2 / 3!) * (f'''(c1) + f'''(c2))
// for some c in [x-h, x+h]. For y = x^2, f'''(x) = 0, so error is 0
TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD1Central)
{
    for (size_t i = 1; i < _ys.size() - 1; ++i) {
        volcart::testing::ExpectNear(D1Central(_ys, i), 2 * _xs[i], percentTol);
    }
}

TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD1FivePointStencil)
{
    for (size_t i = 2; i < _ys.size() - 2; ++i) {
        volcart::testing::ExpectNear(
            D1FivePointStencil(_ys, i), 2 * _xs[i], percentTol);
    }
}

TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD1At)
{
    for (size_t i = 0; i < _ys.size(); ++i) {
        if (i == 0) {
            volcart::testing::ExpectNear(
                D1At(_ys, i), 2 * _xs[i] + 1, percentTol);
        } else if (i == _ys.size() - 1) {
            volcart::testing::ExpectNear(
                D1At(_ys, i), 2 * _xs[i] - 1, percentTol);
        } else {
            volcart::testing::ExpectNear(D1At(_ys, i), 2 * _xs[i], percentTol);
        }
    }
}

TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD1)
{
    auto res = D1(_ys);
    for (size_t i = 0; i < _ys.size(); ++i) {
        if (i == 0) {
            volcart::testing::ExpectNear(res[i], 2 * _xs[i] + 1, percentTol);
        } else if (i == _ys.size() - 1) {
            volcart::testing::ExpectNear(res[i], 2 * _xs[i] - 1, percentTol);
        } else {
            volcart::testing::ExpectNear(res[i], 2 * _xs[i], percentTol);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// y = x^2 D2 testing

TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD2Forward)
{
    for (size_t i = 0; i < _ys.size() - 2; ++i) {
        volcart::testing::ExpectNear(D2Forward(_ys, i), 2, percentTol);
    }
}

// Backward difference error is given by:
//     error = (h / 2) * f''(c)
// for some c in [x, x+h]. For y = x^2, f''(x) = 2, so error is +1
TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD2Backward)
{
    for (size_t i = 2; i < _ys.size(); ++i) {
        volcart::testing::ExpectNear(D2Backward(_ys, i), 2, percentTol);
    }
}

// Central difference error is given by:
//     error = (h^2 / 3!) * (f'''(c1) + f'''(c2))
// for some c in [x-h, x+h]. For y = x^2, f'''(x) = 0, so error is 0
TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD2Central)
{
    for (size_t i = 1; i < _ys.size() - 1; ++i) {
        volcart::testing::ExpectNear(D2Central(_ys, i), 2, percentTol);
    }
}

TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD2FivePointStencil)
{
    for (size_t i = 2; i < _ys.size() - 2; ++i) {
        volcart::testing::ExpectNear(D2FivePointStencil(_ys, i), 2, percentTol);
    }
}

TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD2At)
{
    for (size_t i = 0; i < _ys.size(); ++i) {
        volcart::testing::ExpectNear(D2At(_ys, i), 2, percentTol);
    }
}

TEST_F(YEqualsXSquaredDoubleVectorFixture, YEqualsXSquaredD2)
{
    auto res = D2(_ys);
    for (size_t i = 0; i < _ys.size(); ++i) {
        volcart::testing::ExpectNear(res[i], 2, percentTol);
    }
}