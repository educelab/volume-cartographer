#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <vector>

#include "vc/segmentation/lrps/FittedCurve.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart::segmentation;

// Global float comparison percent tolerance
static const double tol = 0.01;  // %

auto makeTVals(std::size_t count) -> std::vector<double>;

// Fixture for a constant line y = 1
struct ConstantFittedCurve {

    std::size_t _pointCount;
    double _yConstant;
    FittedCurve _curve;

    ConstantFittedCurve(std::size_t pointCount, double yConstant)
        : _pointCount(pointCount), _yConstant(yConstant)
    {
        std::vector<double> xs(pointCount), ys(pointCount);
        std::iota(std::begin(xs), std::end(xs), 0.0);
        std::fill(std::begin(ys), std::end(ys), yConstant);
        std::vector<Voxel> vs(pointCount);
        for (std::size_t i = 0; i < pointCount; ++i) {
            vs[i] = {xs[i], ys[i], -1};
        }
        _curve = FittedCurve(vs, -1);
    }
};

// Generates a curve representing a circle with radius 'r' centered at (0, 0)
struct CircleFittedCurve {

    std::size_t _degreeStep;
    double _radius;
    std::size_t _pointCount;
    FittedCurve _curve;

    CircleFittedCurve(double radius, std::size_t degreeStep = 1)
        : _degreeStep(degreeStep), _radius(radius), _pointCount(0)
    {
        std::vector<Voxel> vs;
        for (std::size_t deg = 0; deg < 360; deg += degreeStep) {
            vs.emplace_back(
                radius * std::cos(deg * M_PI / 180.0),
                radius * std::sin(deg * M_PI / 180.0), -1);
        };
        _pointCount = vs.size();
        _curve = FittedCurve(vs, -1);
    }
};

TEST(ConstantFittedCurve, CheckIncreasingXAndConstantYFromInitialCurve)
{
    const double yConstant = 1;
    auto curve = ConstantFittedCurve(20, yConstant)._curve;
    volcart::testing::ExpectNear(curve(0)(1), yConstant, tol);
    for (std::size_t i = 1; i < curve.size(); ++i) {
        auto prev = curve(i - 1);
        auto curr = curve(i);
        EXPECT_TRUE(prev(0) < curr(0));
        volcart::testing::ExpectNear(curr(1), yConstant, tol);
    }
}

TEST(
    ConstantFittedCurve,
    CheckIncreasingXAndConstantYAndEquidistancePointsFromEvenlySpacedCurve)
{
    const double yConstant = 1;
    auto curve = ConstantFittedCurve(20, yConstant)._curve;
    auto evenlySpaced = curve.evenlySpacePoints();

    double firstDiff = cv::norm(curve(1), curve(0));
    volcart::testing::ExpectNear(evenlySpaced[0](1), yConstant, tol);
    for (std::size_t i = 1; i < evenlySpaced.size(); ++i) {
        volcart::testing::ExpectNear(evenlySpaced[i](1), yConstant, tol);
        volcart::testing::ExpectNear(
            cv::norm(evenlySpaced[i], evenlySpaced[i - 1]), firstDiff, tol);
    }
}

TEST(ConstantFittedCurve, ReadIncreasingXAndConstantYFrom4xSampledCurve)
{
    const double yConstant = 1;
    const std::size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    auto superSampled = curve.sample(npoints * 4);

    volcart::testing::ExpectNear(superSampled[0](1), yConstant, tol);
    for (std::size_t i = 1; i < superSampled.size(); ++i) {
        auto curr = superSampled[i];
        auto prev = superSampled[i - 1];
        volcart::testing::ExpectNear(curr(1), yConstant, tol);
        EXPECT_TRUE(curr(0) > prev(0));
    }
}

TEST(ConstantFittedCurve, VerifyWeGetTheSamePointsBackFor100PercentResample)
{
    const double yConstant = 1;
    const std::size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    auto beforeResamplePoints = curve.points();
    curve.resample(1.0);
    EXPECT_EQ(curve.points().size(), beforeResamplePoints.size());
    for (std::size_t i = 0; i < npoints; ++i) {
        volcart::testing::ExpectNear(
            cv::norm(beforeResamplePoints[i]), cv::norm(curve(i)), tol);
    }
}

TEST(ConstantFittedCurve, EvalReturnsIncreasingXAndConstantY)
{
    const double yConstant = 1;
    const std::size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    auto ts = makeTVals(50);

    volcart::testing::ExpectNear(curve.eval(ts[0])(1), yConstant, tol);
    for (size_t i = 1; i < ts.size(); ++i) {
        auto prev = curve.eval(ts[i - 1]);
        auto curr = curve.eval(ts[i]);
        volcart::testing::ExpectNear(curr(1), yConstant, tol);
        EXPECT_TRUE(curr(0) > prev(0));
    }
}

TEST(ConstantFittedCurve, ConstantCurveHasZeroCurvature)
{
    const double yConstant = 1;
    const std::size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    for (auto k : curve.curvature()) {
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, k, 1e-6);
    }
}

TEST(ConstantFittedCurve, ConstantCurveHasArcLengthOfNPointsMinusOne)
{
    const double yConstant = 1;
    const std::size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    volcart::testing::ExpectNear(curve.arclength(), double(npoints - 1), tol);
}

// Testing CircleFittedCurve

TEST(CircleFittedCurve, CircleCurveHasConstantCurvature)
{
    const double radius = 10;
    auto curve = CircleFittedCurve(radius)._curve;
    auto ks = curve.curvature();
    for (auto k : ks) {
        // Note: increasing tol by 2x to cover start/endpoints that are less
        // accurate than middle points
        volcart::testing::ExpectNear(k, ks.front(), tol * 2);
    }
}

TEST(CircleFittedCurve, CircleCurveHasArcLength2PiR)
{
    const double radius = 10;
    auto curve = CircleFittedCurve(radius)._curve;
    // XXX This is kind of an arbitrary tolerance. There will be some error
    // since it's a linear approx of the circumference, but I'm not sure what a
    // proper error bound should be.
    volcart::testing::ExpectNear(curve.arclength(), 2 * M_PI * radius, 0.3);
}

TEST(CircleFittedCurve, EvenlySampledCircleCurveHasEvenlySpacedPoints)
{
    const double radius = 10;
    auto curve = CircleFittedCurve(radius)._curve;
    auto evenlySpaced = curve.evenlySpacePoints();
    auto firstDiff = cv::norm(evenlySpaced[1], evenlySpaced[0]);
    for (std::size_t i = 1; i < evenlySpaced.size(); ++i) {
        volcart::testing::ExpectNear(
            cv::norm(evenlySpaced[i], evenlySpaced[i - 1]), firstDiff, tol);
    }
}

auto makeTVals(std::size_t count) -> std::vector<double>
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