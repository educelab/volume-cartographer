#define BOOST_TEST_MODULE LocalResliceParticleSimDerivative

#include <iostream>
#include <numeric>

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <opencv2/core.hpp>

#include "segmentation/lrps/derivative.h"

using Voxel = cv::Vec3d;
using namespace volcart::segmentation;

static const double percentTol = 0.01;  // %
static const double smallTol = 1e-6;

struct ConstantDoubleVectorFixture {
    std::vector<double> _v;

    ConstantDoubleVectorFixture() : _v(20)
    {
        std::fill(std::begin(_v), std::end(_v), 1.0);
    }
};

struct YEqualsXSquaredDoubleVectorFixture {
    std::vector<double> _xs;
    std::vector<double> _ys;

    YEqualsXSquaredDoubleVectorFixture() : _xs(20)
    {
        int32_t start = 2;
        std::iota(std::begin(_xs), std::end(_xs), start);
        for (auto x : _xs) {
            _ys.push_back(x * x);
        }
    }
};

////////////////////////////////////////////////////////////////////////////////
// constant d1 testing
// Note: Using BOOST_CHECK_SMALL since _v[i] result is 0. If we use
// CHECK_CLOSE, it will fail because 0 * percent = 0
BOOST_FIXTURE_TEST_SUITE(ConstantD1DoubleTest, ConstantDoubleVectorFixture)

BOOST_AUTO_TEST_CASE(ConstantD1Forward)
{
    for (size_t i = 0; i < _v.size() - 1; ++i) {
        BOOST_CHECK_SMALL(d1Forward(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD1Backward)
{
    for (size_t i = 1; i < _v.size(); ++i) {
        BOOST_CHECK_SMALL(d1Backward(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD1Central)
{
    for (size_t i = 1; i < _v.size() - 1; ++i) {
        BOOST_CHECK_SMALL(d1Central(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD1FivePointStencil)
{
    for (size_t i = 2; i < _v.size() - 2; ++i) {
        BOOST_CHECK_SMALL(d1FivePointStencil(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD1At)
{
    for (size_t i = 0; i < _v.size(); ++i) {
        BOOST_CHECK_SMALL(d1At(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD1)
{
    for (auto d : d1(_v)) {
        BOOST_CHECK_SMALL(d, smallTol);
    }
}

BOOST_AUTO_TEST_SUITE_END()

////////////////////////////////////////////////////////////////////////////////
// constant d2 testing
BOOST_FIXTURE_TEST_SUITE(ConstantD2DoubleTest, ConstantDoubleVectorFixture)

BOOST_AUTO_TEST_CASE(ConstantD2Forward)
{
    for (size_t i = 0; i < _v.size() - 2; ++i) {
        BOOST_CHECK_SMALL(d2Forward(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD2Backward)
{
    for (size_t i = 2; i < _v.size(); ++i) {
        BOOST_CHECK_SMALL(d2Backward(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD2Central)
{
    for (size_t i = 1; i < _v.size() - 1; ++i) {
        BOOST_CHECK_SMALL(d2Central(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD2FivePointStencil)
{
    for (size_t i = 2; i < _v.size() - 2; ++i) {
        BOOST_CHECK_SMALL(d2FivePointStencil(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD2At)
{
    for (size_t i = 0; i < _v.size(); ++i) {
        BOOST_CHECK_SMALL(d2At(_v, i), smallTol);
    }
}

BOOST_AUTO_TEST_CASE(ConstantD2)
{
    for (auto d : d2(_v)) {
        BOOST_CHECK_SMALL(d, smallTol);
    }
}

BOOST_AUTO_TEST_SUITE_END()

////////////////////////////////////////////////////////////////////////////////
// y = x^2 d1 testing
BOOST_FIXTURE_TEST_SUITE(
    YEqualsXSquaredD1Test, YEqualsXSquaredDoubleVectorFixture)

// Forward difference error is given by:
//     error = - (h / 2) * f''(c)
// for some c in [x, x+h]. For y = x^2, f''(x) = 2, so error is -1
BOOST_AUTO_TEST_CASE(YEqualsXSquaredD1Forward)
{
    for (size_t i = 0; i < _ys.size() - 1; ++i) {
        BOOST_CHECK_CLOSE(d1Forward(_ys, i), 2 * _xs[i] + 1, percentTol);
    }
}

// Backward difference error is given by:
//     error = (h / 2) * f''(c)
// for some c in [x, x+h]. For y = x^2, f''(x) = 2, so error is +1
BOOST_AUTO_TEST_CASE(YEqualsXSquaredD1Backward)
{
    for (size_t i = 1; i < _ys.size(); ++i) {
        BOOST_CHECK_CLOSE(d1Backward(_ys, i), 2 * _xs[i] - 1, percentTol);
    }
}

// Central difference error is given by:
//     error = (h^2 / 3!) * (f'''(c1) + f'''(c2))
// for some c in [x-h, x+h]. For y = x^2, f'''(x) = 0, so error is 0
BOOST_AUTO_TEST_CASE(YEqualsXSquaredD1Central)
{
    for (size_t i = 1; i < _ys.size() - 1; ++i) {
        BOOST_CHECK_CLOSE(d1Central(_ys, i), 2 * _xs[i], percentTol);
    }
}

BOOST_AUTO_TEST_CASE(YEqualsXSquaredD1FivePointStencil)
{
    for (size_t i = 2; i < _ys.size() - 2; ++i) {
        BOOST_CHECK_CLOSE(d1FivePointStencil(_ys, i), 2 * _xs[i], percentTol);
    }
}

BOOST_AUTO_TEST_CASE(YEqualsXSquaredD1At)
{
    for (size_t i = 0; i < _ys.size(); ++i) {
        if (i == 0) {
            BOOST_CHECK_CLOSE(d1At(_ys, i), 2 * _xs[i] + 1, percentTol);
        } else if (i == _ys.size() - 1) {
            BOOST_CHECK_CLOSE(d1At(_ys, i), 2 * _xs[i] - 1, percentTol);
        } else {
            BOOST_CHECK_CLOSE(d1At(_ys, i), 2 * _xs[i], percentTol);
        }
    }
}

BOOST_AUTO_TEST_CASE(YEqualsXSquaredD1)
{
    auto res = d1(_ys);
    for (size_t i = 0; i < _ys.size(); ++i) {
        if (i == 0) {
            BOOST_CHECK_CLOSE(res[i], 2 * _xs[i] + 1, percentTol);
        } else if (i == _ys.size() - 1) {
            BOOST_CHECK_CLOSE(res[i], 2 * _xs[i] - 1, percentTol);
        } else {
            BOOST_CHECK_CLOSE(res[i], 2 * _xs[i], percentTol);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()

////////////////////////////////////////////////////////////////////////////////
// y = x^2 d2 testing
BOOST_FIXTURE_TEST_SUITE(
    YEqualsXSquaredD2Test, YEqualsXSquaredDoubleVectorFixture)

BOOST_AUTO_TEST_CASE(YEqualsXSquaredD2Forward)
{
    for (size_t i = 0; i < _ys.size() - 2; ++i) {
        BOOST_CHECK_CLOSE(d2Forward(_ys, i), 2, percentTol);
    }
}

// Backward difference error is given by:
//     error = (h / 2) * f''(c)
// for some c in [x, x+h]. For y = x^2, f''(x) = 2, so error is +1
BOOST_AUTO_TEST_CASE(YEqualsXSquaredD2Backward)
{
    for (size_t i = 2; i < _ys.size(); ++i) {
        BOOST_CHECK_CLOSE(d2Backward(_ys, i), 2, percentTol);
    }
}

// Central difference error is given by:
//     error = (h^2 / 3!) * (f'''(c1) + f'''(c2))
// for some c in [x-h, x+h]. For y = x^2, f'''(x) = 0, so error is 0
BOOST_AUTO_TEST_CASE(YEqualsXSquaredD2Central)
{
    for (size_t i = 1; i < _ys.size() - 1; ++i) {
        BOOST_CHECK_CLOSE(d2Central(_ys, i), 2, percentTol);
    }
}

BOOST_AUTO_TEST_CASE(YEqualsXSquaredD2FivePointStencil)
{
    for (size_t i = 2; i < _ys.size() - 2; ++i) {
        BOOST_CHECK_CLOSE(d2FivePointStencil(_ys, i), 2, percentTol);
    }
}

BOOST_AUTO_TEST_CASE(YEqualsXSquaredD2At)
{
    for (size_t i = 0; i < _ys.size(); ++i) {
        BOOST_CHECK_CLOSE(d2At(_ys, i), 2, percentTol);
    }
}

BOOST_AUTO_TEST_CASE(YEqualsXSquaredD2)
{
    auto res = d2(_ys);
    for (size_t i = 0; i < _ys.size(); ++i) {
        BOOST_CHECK_CLOSE(res[i], 2, percentTol);
    }
}

BOOST_AUTO_TEST_SUITE_END()
