#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

#include "vc/segmentation/lrps/CubicSplineMT.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart::segmentation;

// Percent tolerance for floating-point comparisons
static constexpr double kTol = 0.01;  // 1%

// Build a horizontal line y=c from n evenly-spaced x knots
static auto MakeConstantSpline(std::size_t n, double yConst) -> CubicSplineMT
{
    std::vector<double> xs(n), ys(n);
    std::iota(xs.begin(), xs.end(), 0.0);
    std::fill(ys.begin(), ys.end(), yConst);
    return CubicSplineMT(xs, ys);
}

// Build a parabola y=x² from n evenly-spaced knots starting at xStart
static auto MakeParabolicSpline(std::size_t n, double xStart) -> CubicSplineMT
{
    std::vector<double> xs(n), ys(n);
    std::iota(xs.begin(), xs.end(), xStart);
    std::transform(
        xs.begin(), xs.end(), ys.begin(), [](double x) { return x * x; });
    return CubicSplineMT(xs, ys);
}

////////////////////////////////////////////////////////////////////////////////
// Construction

TEST(CubicSplineMTTest, ConstructFromXYPairs)
{
    // Should not throw or crash
    auto s = MakeConstantSpline(10, 1.0);
    const auto p0 = s(0.0);
    const auto p1 = s(1.0);
    // x at t=1 should be greater than x at t=0
    EXPECT_GT(p1(0), p0(0));
}

TEST(CubicSplineMTTest, ConstructFromVoxelVector)
{
    std::vector<Voxel> vs;
    for (int i = 0; i < 10; ++i) {
        vs.emplace_back(static_cast<double>(i), 5.0, 0.0);
    }
    CubicSplineMT s(vs);
    // y should be constant ≈ 5
    for (double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        volcart::testing::ExpectNear(s(t)(1), 5.0, kTol);
    }
}

TEST(CubicSplineMTTest, CopyConstructionAndAssignment)
{
    auto orig = MakeConstantSpline(10, 3.0);
    CubicSplineMT copy(orig);
    CubicSplineMT assigned;
    assigned = orig;

    for (double t : {0.0, 0.5, 1.0}) {
        volcart::testing::ExpectNear(copy(t)(1), orig(t)(1), kTol);
        volcart::testing::ExpectNear(assigned(t)(1), orig(t)(1), kTol);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Constant spline (y = c): y-coordinate should be constant at any t

TEST(CubicSplineMTTest, ConstantYIsPreservedAtEndpoints)
{
    constexpr double kY = 7.5;
    auto s = MakeConstantSpline(15, kY);
    volcart::testing::ExpectNear(s(0.0)(1), kY, kTol);
    volcart::testing::ExpectNear(s(1.0)(1), kY, kTol);
}

TEST(CubicSplineMTTest, ConstantYIsPreservedThroughout)
{
    constexpr double kY = 4.0;
    auto s = MakeConstantSpline(20, kY);
    for (int i = 0; i <= 20; ++i) {
        const double t = static_cast<double>(i) / 20.0;
        volcart::testing::ExpectNear(s(t)(1), kY, kTol);
    }
}

TEST(CubicSplineMTTest, ConstantSplineXIsMonotonicallyIncreasing)
{
    auto s = MakeConstantSpline(20, 1.0);
    double prevX = s(0.0)(0);
    for (int i = 1; i <= 20; ++i) {
        const double t = static_cast<double>(i) / 20.0;
        const double x = s(t)(0);
        EXPECT_GT(x, prevX);
        prevX = x;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Parabolic spline (y ≈ x²): sampled points should satisfy y ≈ x²

TEST(CubicSplineMTTest, ParabolicSplineApproximatesYEqualsXSquared)
{
    // Use 20 knots over [0, 19]; natural spline through y=x² is exact for
    // polynomials of degree ≤ 3, so we expect tight agreement.
    auto s = MakeParabolicSpline(20, 0.0);
    for (int i = 1; i <= 18; ++i) {
        const double t = static_cast<double>(i) / 19.0;
        const auto p = s(t);
        // Looser tolerance (5%) since arc-length parameterization shifts
        // the sampling slightly away from integer x positions
        volcart::testing::ExpectNear(p(0) * p(0), p(1), 5.0);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Edge cases

TEST(CubicSplineMTTest, TwoKnotsLinearInterpolation)
{
    // Two knots → linear segment, no cubic terms
    CubicSplineMT s({0.0, 1.0}, {0.0, 2.0});
    // midpoint should be near (0.5, 1.0)
    const auto mid = s(0.5);
    volcart::testing::ExpectNear(mid(0), 0.5, kTol);
    volcart::testing::ExpectNear(mid(1), 1.0, kTol);
}

TEST(CubicSplineMTTest, MultiWindowPath)
{
    // >100 knots triggers the parallel multi-window code path in FitSplineMT
    constexpr std::size_t kN = 150;
    auto s = MakeConstantSpline(kN, 2.0);
    for (int i = 0; i <= 10; ++i) {
        const double t = static_cast<double>(i) / 10.0;
        volcart::testing::ExpectNear(s(t)(1), 2.0, kTol);
    }
}
