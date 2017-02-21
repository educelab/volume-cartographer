#define BOOST_TEST_MODULE LocalResliceParticleSimEnergyMetrics

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc/segmentation/lrps/EnergyMetrics.hpp"

using namespace volcart::segmentation;

// Testing constants
const double tolperc = 0.01;
const double tol = 1e-9;
const int kDefaultWindowSize = 3;
const int kDefaultIndex = 0;
const double kDefaultAlpha = 1.0 / 3.0;
const double kDefaultBeta = 1.0 / 3.0;
const double kDefaultDelta = 1.0 / 3.0;
const double kDefaultK1 = 0.5;
const double kDefaultK2 = 0.5;

struct EmptyFittedCurve {
    FittedCurve _curve;

    EmptyFittedCurve() {}
};

// Fixture for a constant line y = 1
struct ConstantFittedCurve {
    FittedCurve _curve;

    ConstantFittedCurve()
    {
        size_t pointCount = 10;
        double yConstant = 1.0;
        std::vector<double> xs(pointCount), ys(pointCount);
        std::iota(std::begin(xs), std::end(xs), 0.0);
        std::fill(std::begin(ys), std::end(ys), yConstant);
        std::vector<Voxel> vs(pointCount);
        for (size_t i = 0; i < pointCount; ++i) {
            vs[i] = {xs[i], ys[i], 0};
        }
        _curve = FittedCurve(vs, 0);
    }
};

////////////////////////////////////////////////////////////////////////////////
// Check methods return expected values for an empty curve
BOOST_FIXTURE_TEST_SUITE(EmptyCurveEnergyMetrics, EmptyFittedCurve)

BOOST_AUTO_TEST_CASE(ActiveContourEnergyWithEmptyCurve)
{
    BOOST_CHECK_SMALL(
        EnergyMetrics::ActiveContourInternal(_curve, kDefaultK1, kDefaultK2),
        tol);
}

BOOST_AUTO_TEST_CASE(ActiveContourEnergyWithEmptyCurveAndDifferntParams)
{
    const double k1 = 0.1;
    const double k2 = 0.1;
    BOOST_CHECK_SMALL(
        EnergyMetrics::ActiveContourInternal(_curve, k1, k2), tol);
}

BOOST_AUTO_TEST_CASE(AbsCurvatureSumWithEmptyCurve)
{
    BOOST_CHECK_SMALL(EnergyMetrics::AbsCurvatureSum(_curve), tol);
}

BOOST_AUTO_TEST_CASE(LocalWindowedArcLengthWithEmptyCurve)
{
    BOOST_CHECK_SMALL(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, kDefaultWindowSize),
        tol);
}

BOOST_AUTO_TEST_CASE(LocalWindowedArcLengthWithEmptyCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    BOOST_CHECK_SMALL(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, negativeWindow),
        tol);
}

BOOST_AUTO_TEST_CASE(LocalWindowedArcLengthWithEmptyCurveAndLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    BOOST_CHECK_SMALL(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, largeWindow),
        tol);
}

BOOST_AUTO_TEST_CASE(
    LocalWindowedArcLengthWithEmptyCurveAndNegativeWindowAndOOBIndex)
{
    const int negativeIndex = -1;
    const int negativeWindow = -1;
    BOOST_CHECK_SMALL(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, negativeIndex, negativeWindow),
        tol);
}

BOOST_AUTO_TEST_CASE(WindowedArcLengthWithEmptyCurve)
{
    const int zeroWindow = 0;
    BOOST_CHECK_SMALL(
        EnergyMetrics::WindowedArcLength(_curve, zeroWindow), tol);
}

BOOST_AUTO_TEST_CASE(WindowedArcLengthWithEmptyCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    BOOST_CHECK_SMALL(
        EnergyMetrics::WindowedArcLength(_curve, negativeWindow), tol);
}

BOOST_AUTO_TEST_CASE(WindowedArcLengthWithEmptyCurveAndTooLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    BOOST_CHECK_SMALL(
        EnergyMetrics::WindowedArcLength(_curve, largeWindow), tol);
}

BOOST_AUTO_TEST_CASE(WindowedArcLengthWithEmptyCurveAndNonZeroWindowSize)
{
    const int nonZeroWindow = kDefaultWindowSize;
    BOOST_CHECK_SMALL(
        EnergyMetrics::WindowedArcLength(_curve, nonZeroWindow), tol);
}

BOOST_AUTO_TEST_CASE(TotalEnergyWithEmptyCurve)
{
    BOOST_CHECK_SMALL(
        EnergyMetrics::TotalEnergy(
            _curve, kDefaultAlpha, kDefaultK1, kDefaultK2, kDefaultBeta,
            kDefaultDelta),
        tol);
}

BOOST_AUTO_TEST_SUITE_END()

////////////////////////////////////////////////////////////////////////////////
// Test energy functions with a constant curve
BOOST_FIXTURE_TEST_SUITE(ConstantCurveEnergyMetrics, ConstantFittedCurve)

BOOST_AUTO_TEST_CASE(ActiveContourEnergyWithConstantCurve)
{
    const double expected = kDefaultK1 / 2;
    auto result =
        EnergyMetrics::ActiveContourInternal(_curve, kDefaultK1, kDefaultK2);
    BOOST_CHECK_CLOSE(result, expected, tolperc);
}

BOOST_AUTO_TEST_CASE(ActiveContourEnergyWithConstantCurveAndDifferntParams)
{
    const double k1 = 0.1;
    const double k2 = 0.1;
    const double expected = k1 / 2;
    auto result = EnergyMetrics::ActiveContourInternal(_curve, k1, k2);
    BOOST_CHECK_CLOSE(result, expected, tolperc);
}

BOOST_AUTO_TEST_CASE(AbsCurvatureSumWithConstantCurve)
{
    auto result = EnergyMetrics::AbsCurvatureSum(_curve);
    BOOST_CHECK_SMALL(result, tol);
}

BOOST_AUTO_TEST_CASE(
    LocalWindowedArcLengthWithConstantCurveAndDefaultWindowSize)
{
    const double expected = kDefaultWindowSize / 2 * 2;
    auto result = EnergyMetrics::LocalWindowedArcLength(
        _curve, kDefaultIndex, kDefaultWindowSize);
    BOOST_CHECK_CLOSE(result, expected, tolperc);
}

BOOST_AUTO_TEST_CASE(
    LocalWindowedArcLengthWithConstantCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    BOOST_CHECK_THROW(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, negativeWindow),
        std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(LocalWindowedArcLengthWithConstantCurveAndLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    BOOST_CHECK_THROW(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, largeWindow),
        std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(
    LocalWindowedArcLengthWithConstantCurveAndNegativeWindowAndOOBIndex)
{
    const int negativeIndex = -1;
    const int negativeWindow = -1;
    BOOST_CHECK_THROW(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, negativeIndex, negativeWindow),
        std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(WindowedArcLengthWithConstantCurveAndDefaultWindowSize)
{
    const int expected = kDefaultWindowSize / 2 * 2;
    auto result = EnergyMetrics::WindowedArcLength(_curve, kDefaultWindowSize);
    BOOST_CHECK_CLOSE(result, expected, tolperc);
}

BOOST_AUTO_TEST_CASE(WindowedArcLengthWithConstantCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    BOOST_CHECK_THROW(
        EnergyMetrics::WindowedArcLength(_curve, negativeWindow),
        std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(WindowedArcLengthWithConstantCurveAndTooLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    BOOST_CHECK_THROW(
        EnergyMetrics::WindowedArcLength(_curve, largeWindow),
        std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(TotalEnergyWithConstantCurveAndDefaultValues)
{
    const double expectedActiveContourSum = kDefaultK1 / 2;
    const double expectedAbsCurvatureSum = 0;
    const double expectedWindowedArcLengthSum = kDefaultWindowSize / 2 * 2;
    const double expected = kDefaultAlpha * expectedActiveContourSum +
                            kDefaultBeta * expectedAbsCurvatureSum +
                            kDefaultDelta * expectedWindowedArcLengthSum;
    auto result = EnergyMetrics::TotalEnergy(
        _curve, kDefaultAlpha, kDefaultK1, kDefaultK2, kDefaultBeta,
        kDefaultDelta);
    BOOST_CHECK_CLOSE(result, expected, tolperc);
}

BOOST_AUTO_TEST_SUITE_END()
