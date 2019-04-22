#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

#include "vc/segmentation/lrps/Spline.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart::segmentation;

// Global float comparison percent tolerance
static const double floatComparePercentTolerance = 0.01;  // %

std::vector<double> generateTVals(size_t count);

// Fixture for a spline parameterizing y = 1
struct ConstantCubicSpline {

    size_t _knotCount;
    double _yConstant;
    CubicSpline<double> _spline;

    ConstantCubicSpline(size_t knotCount, double yConstant)
        : _knotCount(knotCount), _yConstant(yConstant), _spline(makeSpline())
    {
        std::vector<double> xs(knotCount), ys(knotCount);
        std::iota(std::begin(xs), std::end(xs), 0.0);
        std::fill(std::begin(ys), std::end(ys), yConstant);
        _spline = CubicSpline<double>(xs, ys);
    }

    decltype(_spline) makeSpline()
    {
        std::vector<double> xs(_knotCount), ys(_knotCount);
        std::iota(std::begin(xs), std::end(xs), 0.0);
        std::fill(std::begin(ys), std::end(ys), _yConstant);
        return CubicSpline<double>(xs, ys);
    }
};

// Fixture for a spline parameterizing y = x^2
struct ParabolicCubicSpline {

    size_t _knotCount;
    double _splineStart;
    CubicSpline<double> _spline;

    ParabolicCubicSpline(size_t knotCount, double splineStart)
        : _knotCount(knotCount)
        , _splineStart(splineStart)
        , _spline(makeSpline())
    {
    }

    decltype(_spline) makeSpline()
    {
        // Fill x values
        std::vector<double> xs(_knotCount), ys(_knotCount);
        std::iota(std::begin(xs), std::end(xs), _splineStart);

        // Fill y values
        std::transform(
            std::begin(xs), std::end(xs), std::begin(ys),
            [](double e) { return e * e; });

        return CubicSpline<double>(xs, ys);
    }
};

////////////////////////////////////////////////////////////////////////////////
// Test tval generation
// Note: use BOOST_REQUIRE_* here so we don't go on to other tests if this check
// fails since those tests rely on this functionality
TEST(CubicSplineTest, CanGenerateCorrectTValues)
{
    // Create N t-values to evaluate the spline at and make sure we get a
    // constant back y-value back
    auto ts = generateTVals(20);

    // 1. Check neighbor differences - should be equal
    // Start at 2 so we skip the first pair
    double firstDiff = ts[1] - ts[0];
    for (size_t i = 2; i < ts.size(); ++i) {
        volcart::testing::AssertNear(
            firstDiff, ts[i] - ts[i - 1], floatComparePercentTolerance);
    }
    for (const auto t : ts) {
        ASSERT_TRUE(t >= 0 && t <= 1);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Test constant spline
TEST(CubicSplineTest, YEqualsOne)
{
    size_t count = 10;
    ConstantCubicSpline s(count, 1.0);

    // auto tol = btt::percent_tolerance(floatComparePercentTolerance);
    // auto eps = std::numeric_limits<double>::epsilon();

    auto ts = generateTVals(count);
    for (size_t i = 1; i < ts.size(); ++i) {
        auto p0 = s._spline(ts[i - 1]);
        auto p1 = s._spline(ts[i]);

        // Check that the x values are both between the specified range
        // XXX Disabled until I can find a better way to validate that these
        // numbers are in the correct range
        /*
        BOOST_CHECK(std::abs(p0(0)) >= 0 + eps &&
                    std::abs(p0(0)) <= count - 1 + eps);
        BOOST_CHECK(std::abs(p1(0)) >= 0 + eps &&
                    std::abs(p1(0)) <= count - 1 + eps);
                    */

        // Check that p0 < p1 in x-domain
        EXPECT_TRUE(p0(0) < p1(0));

        // Check that our y-value is constant
        volcart::testing::ExpectNear(
            p0(1), s._yConstant, floatComparePercentTolerance);
        volcart::testing::ExpectNear(
            p1(1), s._yConstant, floatComparePercentTolerance);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Test parabolic spline
TEST(CubicSplineTest, YEqualsXSquaredValues)
{
    size_t count = 20;
    ParabolicCubicSpline s(count, -10);
    auto ts = generateTVals(count * 2);
    for (auto t : ts) {
        auto p = s._spline(t);
        volcart::testing::ExpectNear(p(0) * p(0), p(1), 2.0);
    }
}

std::vector<double> generateTVals(size_t count)
{
    std::vector<double> ts(count);
    ts.front() = 0;
    double sum = 0;
    std::generate(std::begin(ts) + 1, std::end(ts) - 1, [count, &sum]() {
        return sum += 1.0 / (count - 1);
    });
    ts.back() = 1;
    return ts;
}
