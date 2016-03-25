#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE LocalResliceParticleSimFittedCurve

#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "localResliceParticleSim/fittedcurve.h"

using namespace volcart::segmentation;

// Global float comparison percent tolerance
static const double tol = 0.01;  // %

std::vector<double> makeTVals(size_t count);

// Fixture for a constant line y = 1
struct ConstantFittedCurve {
    size_t _pointCount;
    double _yConstant;
    FittedCurve _curve;

    ConstantFittedCurve(size_t pointCount, double yConstant)
        : _pointCount(pointCount), _yConstant(yConstant)
    {
        std::vector<double> xs(pointCount), ys(pointCount);
        std::iota(std::begin(xs), std::end(xs), 0.0);
        std::fill(std::begin(ys), std::end(ys), yConstant);
        std::vector<Voxel> vs(pointCount);
        for (size_t i = 0; i < pointCount; ++i) {
            vs[i] = {xs[i], ys[i], -1};
        }
        _curve = FittedCurve(vs, -1);
    }
};

// Generates a curve representing a circle with radius 'r' centered at (0, 0)
struct CircleFittedCurve {
    size_t _degreeStep;
    double _radius;
    size_t _pointCount;
    FittedCurve _curve;

    CircleFittedCurve(double radius, size_t degreeStep = 1)
        : _degreeStep(degreeStep), _radius(radius), _pointCount(0)
    {
        std::vector<Voxel> vs;
        for (size_t deg = 0; deg < 360; deg += degreeStep) {
            vs.emplace_back(radius * std::cos(deg * M_PI / 180.0),
                            radius * std::sin(deg * M_PI / 180.0), -1);
        };
        _pointCount = vs.size();
        _curve = FittedCurve(vs, -1);
    }
};

BOOST_AUTO_TEST_SUITE(ConstantCurveSuite)

BOOST_AUTO_TEST_CASE(CheckIncreasingXAndConstantYFromInitialCurve)
{
    const double yConstant = 1;
    auto curve = ConstantFittedCurve(20, yConstant)._curve;
    BOOST_CHECK_CLOSE(curve(0)(1), yConstant, tol);
    for (size_t i = 1; i < curve.size(); ++i) {
        auto prev = curve(i - 1);
        auto curr = curve(i);
        BOOST_CHECK(prev(0) < curr(0));
        BOOST_CHECK_CLOSE(curr(1), yConstant, tol);
    }
}

BOOST_AUTO_TEST_CASE(
    CheckIncreasingXAndConstantYAndEquidistancePointsFromEvenlySpacedCurve)
{
    const double yConstant = 1;
    auto curve = ConstantFittedCurve(20, yConstant)._curve;
    auto evenlySpaced = curve.evenlySpacePoints();

    double firstDiff = cv::norm(curve(1), curve(0));
    BOOST_CHECK_CLOSE(evenlySpaced[0](1), yConstant, tol);
    for (size_t i = 1; i < evenlySpaced.size(); ++i) {
        BOOST_CHECK_CLOSE(evenlySpaced[i](1), yConstant, tol);
        BOOST_CHECK_CLOSE(cv::norm(evenlySpaced[i], evenlySpaced[i - 1]),
                          firstDiff, tol);
    }
}

BOOST_AUTO_TEST_CASE(ReadIncreasingXAndConstantYFrom4xSampledCurve)
{
    const double yConstant = 1;
    const size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    auto superSampled = curve.sample(npoints * 4);

    BOOST_CHECK_CLOSE(superSampled[0](1), yConstant, tol);
    for (size_t i = 1; i < superSampled.size(); ++i) {
        auto curr = superSampled[i];
        auto prev = superSampled[i - 1];
        BOOST_CHECK_CLOSE(curr(1), yConstant, tol);
        BOOST_CHECK(curr(0) > prev(0));
    }
}

BOOST_AUTO_TEST_CASE(VerifyWeGetTheSamePointsBackFor100PercentResample)
{
    const double yConstant = 1;
    const size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    auto beforeResamplePoints = curve.points();
    curve.resample(1.0);
    BOOST_CHECK_EQUAL(curve.points().size(), beforeResamplePoints.size());
    for (size_t i = 0; i < npoints; ++i) {
        BOOST_CHECK_CLOSE(cv::norm(beforeResamplePoints[i]), cv::norm(curve(i)),
                          tol);
    }
}

BOOST_AUTO_TEST_CASE(EvalReturnsIncreasingXAndConstantY)
{
    const double yConstant = 1;
    const size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    auto ts = makeTVals(50);

    BOOST_CHECK_CLOSE(curve.eval(ts[0])(1), yConstant, tol);
    for (size_t i = 1; i < ts.size(); ++i) {
        auto prev = curve.eval(ts[i - 1]);
        auto curr = curve.eval(ts[i]);
        BOOST_CHECK_CLOSE(curr(1), yConstant, tol);
        BOOST_CHECK(curr(0) > prev(0));
    }
}

BOOST_AUTO_TEST_CASE(ConstantCurveHasZeroCurvature)
{
    const double yConstant = 1;
    const size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    for (auto k : curve.curvature()) {
        BOOST_CHECK_SMALL(k, 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(ConstantCurveHasArcLengthOfNPointsMinusOne)
{
    const double yConstant = 1;
    const size_t npoints = 20;
    auto curve = ConstantFittedCurve(npoints, yConstant)._curve;
    BOOST_CHECK_CLOSE(curve.arclength(), double(npoints - 1), tol);
}

BOOST_AUTO_TEST_SUITE_END()

// Testing CircleFittedCurve
BOOST_AUTO_TEST_SUITE(CircleCurveSuite)

BOOST_AUTO_TEST_CASE(CircleCurveHasConstantCurvature)
{
    const double radius = 10;
    auto curve = CircleFittedCurve(radius)._curve;
    auto ks = curve.curvature();
    for (auto k : ks) {
        // Note: increasing tol by 2x to cover start/endpoints that are less
        // accurate than middle points
        BOOST_CHECK_CLOSE(k, ks.front(), tol * 2);
    }
}

BOOST_AUTO_TEST_CASE(CircleCurveHasArcLength2PiR)
{
    const double radius = 10;
    auto curve = CircleFittedCurve(radius)._curve;
    // XXX This is kind of an arbitrary tolerance. There will be some error
    // since it's a linear approx of the circumference, but I'm not sure what a
    // proper error bound should be.
    BOOST_CHECK_CLOSE(curve.arclength(), 2 * M_PI * radius, 0.3);
}

BOOST_AUTO_TEST_CASE(EvenlySampledCircleCurveHasEvenlySpacedPoints)
{
    const double radius = 10;
    auto curve = CircleFittedCurve(radius)._curve;
    auto evenlySpaced = curve.evenlySpacePoints();
    auto firstDiff = cv::norm(evenlySpaced[1], evenlySpaced[0]);
    for (size_t i = 1; i < evenlySpaced.size(); ++i) {
        BOOST_CHECK_CLOSE(cv::norm(evenlySpaced[i], evenlySpaced[i - 1]),
                          firstDiff, tol);
    }
}

BOOST_AUTO_TEST_SUITE_END()

std::vector<double> makeTVals(size_t count)
{
    std::vector<double> ts(count);
    ts.front() = 0;
    double sum = 0;
    std::generate(std::begin(ts) + 1, std::end(ts) - 1,
                  [count, &sum]() { return sum += 1.0 / (count - 1); });
    ts.back() = 1;
    return ts;
}
