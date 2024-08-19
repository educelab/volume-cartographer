#include <gtest/gtest.h>

#include <cstddef>
#include <iostream>
#include <numeric>

#include "vc/segmentation/lrps/EnergyMetrics.hpp"
#include "vc/testing/TestingUtils.hpp"

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

class EmptyFittedCurve : public ::testing::Test
{
public:
    FittedCurve _curve;
};

// Fixture for a constant line y = 1
class ConstantFittedCurve : public ::testing::Test
{
public:
    FittedCurve _curve;

    ConstantFittedCurve()
    {
        std::size_t pointCount = 10;
        double yConstant = 1.0;
        std::vector<double> xs(pointCount), ys(pointCount);
        std::iota(std::begin(xs), std::end(xs), 0.0);
        std::fill(std::begin(ys), std::end(ys), yConstant);
        std::vector<Voxel> vs(pointCount);
        for (std::size_t i = 0; i < pointCount; ++i) {
            vs[i] = {xs[i], ys[i], 0};
        }
        _curve = FittedCurve(vs, 0);
    }
};

////////////////////////////////////////////////////////////////////////////////
// Check methods return expected values for an empty curve

TEST_F(EmptyFittedCurve, ActiveContourEnergyWithEmptyCurve)
{
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::ActiveContourInternal(_curve, kDefaultK1, kDefaultK2),
        tol);
}

TEST_F(EmptyFittedCurve, ActiveContourEnergyWithEmptyCurveAndDifferntParams)
{
    const double k1 = 0.1;
    const double k2 = 0.1;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::ActiveContourInternal(_curve, k1, k2), tol);
}

TEST_F(EmptyFittedCurve, AbsCurvatureSumWithEmptyCurve)
{
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE, EnergyMetrics::AbsCurvatureSum(_curve), tol);
}

TEST_F(EmptyFittedCurve, LocalWindowedArcLengthWithEmptyCurve)
{
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, kDefaultWindowSize),
        tol);
}

TEST_F(
    EmptyFittedCurve, LocalWindowedArcLengthWithEmptyCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, negativeWindow),
        tol);
}

TEST_F(EmptyFittedCurve, LocalWindowedArcLengthWithEmptyCurveAndLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, largeWindow),
        tol);
}

TEST_F(
    EmptyFittedCurve,
    LocalWindowedArcLengthWithEmptyCurveAndNegativeWindowAndOOBIndex)
{
    const int negativeIndex = -1;
    const int negativeWindow = -1;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::LocalWindowedArcLength(
            _curve, negativeIndex, negativeWindow),
        tol);
}

TEST_F(EmptyFittedCurve, WindowedArcLengthWithEmptyCurve)
{
    const int zeroWindow = 0;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::WindowedArcLength(_curve, zeroWindow), tol);
}

TEST_F(EmptyFittedCurve, WindowedArcLengthWithEmptyCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::WindowedArcLength(_curve, negativeWindow), tol);
}

TEST_F(EmptyFittedCurve, WindowedArcLengthWithEmptyCurveAndTooLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::WindowedArcLength(_curve, largeWindow), tol);
}

TEST_F(EmptyFittedCurve, WindowedArcLengthWithEmptyCurveAndNonZeroWindowSize)
{
    const int nonZeroWindow = kDefaultWindowSize;
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::WindowedArcLength(_curve, nonZeroWindow), tol);
}

TEST_F(EmptyFittedCurve, TotalEnergyWithEmptyCurve)
{
    EXPECT_PRED_FORMAT2(
        ::testing::DoubleLE,
        EnergyMetrics::TotalEnergy(
            _curve, kDefaultAlpha, kDefaultK1, kDefaultK2, kDefaultBeta,
            kDefaultDelta),
        tol);
}

////////////////////////////////////////////////////////////////////////////////
// Test energy functions with a constant curve

TEST_F(ConstantFittedCurve, ActiveContourEnergyWithConstantCurve)
{
    const double expected = kDefaultK1 / 2;
    auto result =
        EnergyMetrics::ActiveContourInternal(_curve, kDefaultK1, kDefaultK2);
    volcart::testing::ExpectNear(result, expected, tolperc);
}

TEST_F(
    ConstantFittedCurve, ActiveContourEnergyWithConstantCurveAndDifferntParams)
{
    const double k1 = 0.1;
    const double k2 = 0.1;
    const double expected = k1 / 2;
    auto result = EnergyMetrics::ActiveContourInternal(_curve, k1, k2);
    volcart::testing::ExpectNear(result, expected, tolperc);
}

TEST_F(ConstantFittedCurve, AbsCurvatureSumWithConstantCurve)
{
    auto result = EnergyMetrics::AbsCurvatureSum(_curve);
    EXPECT_PRED_FORMAT2(::testing::DoubleLE, result, tol);
}

TEST_F(
    ConstantFittedCurve,
    LocalWindowedArcLengthWithConstantCurveAndDefaultWindowSize)
{
    const double expected = kDefaultWindowSize / 2 * 2;
    auto result = EnergyMetrics::LocalWindowedArcLength(
        _curve, kDefaultIndex, kDefaultWindowSize);
    volcart::testing::ExpectNear(result, expected, tolperc);
}

TEST_F(
    ConstantFittedCurve,
    LocalWindowedArcLengthWithConstantCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    EXPECT_THROW(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, negativeWindow),
        std::invalid_argument);
}

TEST_F(
    ConstantFittedCurve,
    LocalWindowedArcLengthWithConstantCurveAndLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    EXPECT_THROW(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, kDefaultIndex, largeWindow),
        std::invalid_argument);
}

TEST_F(
    ConstantFittedCurve,
    LocalWindowedArcLengthWithConstantCurveAndNegativeWindowAndOOBIndex)
{
    const int negativeIndex = -1;
    const int negativeWindow = -1;
    EXPECT_THROW(
        EnergyMetrics::LocalWindowedArcLength(
            _curve, negativeIndex, negativeWindow),
        std::invalid_argument);
}

TEST_F(
    ConstantFittedCurve, WindowedArcLengthWithConstantCurveAndDefaultWindowSize)
{
    const int expected = kDefaultWindowSize / 2 * 2;
    auto result = EnergyMetrics::WindowedArcLength(_curve, kDefaultWindowSize);
    volcart::testing::ExpectNear(result, expected, tolperc);
}

TEST_F(
    ConstantFittedCurve,
    WindowedArcLengthWithConstantCurveAndNegativeWindowSize)
{
    const int negativeWindow = -1;
    EXPECT_THROW(
        EnergyMetrics::WindowedArcLength(_curve, negativeWindow),
        std::invalid_argument);
}

TEST_F(
    ConstantFittedCurve,
    WindowedArcLengthWithConstantCurveAndTooLargeWindowSize)
{
    const int largeWindow = _curve.size() + 1;
    EXPECT_THROW(
        EnergyMetrics::WindowedArcLength(_curve, largeWindow),
        std::invalid_argument);
}

TEST_F(ConstantFittedCurve, TotalEnergyWithConstantCurveAndDefaultValues)
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
    volcart::testing::ExpectNear(result, expected, tolperc);
}