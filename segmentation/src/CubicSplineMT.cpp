/* This file is licensed under the MIT license. Please see CubicSplineMT.hpp. */

#include "vc/segmentation/lrps/CubicSplineMT.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "vc/core/util/Iteration.hpp"

using namespace volcart;
using namespace volcart::segmentation;
using namespace Eigen;

using Params = std::vector<double>;

namespace
{

template <typename T>
auto linspace(const std::size_t num, const T low, const T high)
    -> std::vector<T>
{
    std::vector<T> v(num);
    const auto step = num > 1 ? (high - low) / static_cast<T>(num - 1) : T{0};
    std::generate(
        v.begin(), v.end(), [n = std::size_t{0}, &low, &step]() mutable {
            return low + step * n++;
        });
    return v;
}

auto Interpolate(const VectorXd& x, const VectorXd& y)
    -> std::tuple<VectorXd, VectorXd, VectorXd, VectorXd>
{
    const auto n = x.size() - 1;
    VectorXd h = x.segment(1, n) - x.segment(0, n);

    // Clamp near-zero intervals to avoid division by zero
    for (int i = 0; i < h.size(); ++i) {
        if (h(i) == 0) {
            h(i) = 1e-8;
        }
    }

    // Natural spline: c[0] = c[n] = 0.
    // Solve the (n-1)×(n-1) tridiagonal system for c[1..n-1] using the
    // Thomas algorithm — O(n) time and O(n) memory.
    VectorXd c = VectorXd::Zero(n + 1);
    if (n > 1) {
        const auto m = n - 1;
        VectorXd diag(m), rhs(m);
        for (int j = 0; j < m; ++j) {
            diag(j) = 2 * (h[j] + h[j + 1]);
            rhs(j) = 3 * ((y[j + 2] - y[j + 1]) / h[j + 1] -
                          (y[j + 1] - y[j]) / h[j]);
        }

        // Forward elimination
        for (int j = 1; j < m; ++j) {
            const double w = h[j] / diag(j - 1);
            diag(j) -= w * h[j];
            rhs(j) -= w * rhs(j - 1);
        }

        // Back substitution (solution maps to c[1..n-1])
        c(m) = rhs(m - 1) / diag(m - 1);
        for (int j = m - 2; j >= 0; --j) {
            c(j + 1) = (rhs(j) - h[j + 1] * c(j + 2)) / diag(j);
        }
    }

    const VectorXd a = y.segment(0, n);
    VectorXd b(n), d(n);
    for (int i = 0; i < n; ++i) {
        b(i) = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3;
        d(i) = (c[i + 1] - c[i]) / (3 * h[i]);
    }

    c.conservativeResize(n);
    return {a, b, c, d};
}

auto FitSplineMT(
    const Params& range,
    const Params& val,
    const std::size_t winSize = 100,
    const std::size_t bufSize = 10)
    -> std::tuple<Params, Params, Params, Params>
{
    const auto n = range.size();
    Params aVec(n, 0.0);
    Params bVec(n, 0.0);
    Params cVec(n, 0.0);
    Params dVec(n, 0.0);

    // Each window writes to a non-overlapping range of the output vectors,
    // so no synchronization is needed.
    const auto nWindows = static_cast<std::ptrdiff_t>(
        std::ceil(static_cast<double>(n) / static_cast<double>(winSize)));
#pragma omp parallel for schedule(dynamic)
    for (std::ptrdiff_t wi = 0; wi < nWindows; ++wi) {
        const auto i = static_cast<std::size_t>(wi) * winSize;
        const auto winEnd = std::min(i + winSize + bufSize, n);
        std::size_t winStart{0};
        if (winEnd > winSize and winEnd - winSize > 2 * bufSize) {
            winStart = winEnd - winSize - 2 * bufSize;
        }
        const auto winStride = static_cast<Index>(winEnd - winStart);

        auto xWin = Eigen::Map<const VectorXd>(&range[winStart], winStride);
        auto yWin = Eigen::Map<const VectorXd>(&val[winStart], winStride);
        auto [a, b, c, d] = Interpolate(xWin, yWin);

        const auto updateStart = i - winStart;
        const auto updateEnd = std::min(i + winSize, n) - winStart;
        std::copy(
            a.data() + updateStart, a.data() + updateEnd, aVec.begin() + i);
        std::copy(
            b.data() + updateStart, b.data() + updateEnd, bVec.begin() + i);
        std::copy(
            c.data() + updateStart, c.data() + updateEnd, cVec.begin() + i);
        std::copy(
            d.data() + updateStart, d.data() + updateEnd, dVec.begin() + i);
    }

    return {aVec, bVec, cVec, dVec};
}

// 5-point Gauss-Legendre arc-length integrand for a cubic spline segment
auto SplineLength(
    const double bX,
    const double cX,
    const double dX,
    const double bY,
    const double cY,
    const double dY,
    const double t0,
    const double tSub0,
    const double tSub1) -> double
{
    // 5-point Gauss-Legendre nodes and weights on [-1, 1]
    static constexpr double kNodes[5] = {
        0.0, -0.5384693101056831, 0.5384693101056831, -0.9061798459386640,
        0.9061798459386640};
    static constexpr double kWeights[5] = {
        0.5688888888888889, 0.4786286704993665, 0.4786286704993665,
        0.2369268850561891, 0.2369268850561891};

    // Transform from [-1, 1] to [tSub0, tSub1]
    const double half = 0.5 * (tSub1 - tSub0);
    const double mid = 0.5 * (tSub1 + tSub0);
    double result{0};
    for (int i = 0; i < 5; ++i) {
        const double t = mid + half * kNodes[i];
        const double dt = t - t0;
        const double dsdx = bX + 2 * cX * dt + 3 * dX * dt * dt;
        const double dsdy = bY + 2 * cY * dt + 3 * dY * dt * dt;
        result += kWeights[i] * std::sqrt(dsdx * dsdx + dsdy * dsdy);
    }
    return std::abs(half * result);
}

// Compute lengths of 2D spline segments
auto SubsegmentLengths(
    const Params& t,
    const Params& bX,
    const Params& cX,
    const Params& dX,
    const Params& bY,
    const Params& cY,
    const Params& dY,
    const std::size_t nSegs = 10) -> std::tuple<Params, Params>
{
    const auto numLengths = nSegs * (t.size() - 1);
    Params subsegLengths;
    subsegLengths.reserve(numLengths);
    Params cumulativeLengths = {0.0};
    cumulativeLengths.reserve(numLengths + 1);

    for (const auto& [i, j] : range2D(t.size() - 1, nSegs)) {
        const auto t0 = t[i];
        const auto t1 = t[i + 1];
        const auto tS = static_cast<double>(j) / static_cast<double>(nSegs);
        const auto tE = static_cast<double>(j + 1) / static_cast<double>(nSegs);
        const auto tSub0 = t0 + (t1 - t0) * tS;
        const auto tSub1 = t0 + (t1 - t0) * tE;
        auto length = SplineLength(
            bX[i], cX[i], dX[i], bY[i], cY[i], dY[i], t0, tSub0, tSub1);
        subsegLengths.emplace_back(length);
        cumulativeLengths.emplace_back(cumulativeLengths.back() + length);
    }

    return {subsegLengths, cumulativeLengths};
}

}  // namespace

// Constructor implementation
CubicSplineMT::CubicSplineMT(const Params& x, const Params& y)
{
    rangeXY_ = linspace(x.size(), 0., static_cast<double>(x.size() - 1));
    std::tie(aX_, bX_, cX_, dX_) = FitSplineMT(rangeXY_, x);
    std::tie(aY_, bY_, cY_, dY_) = FitSplineMT(rangeXY_, y);
    std::tie(subsegLens_, cumuLens_) =
        SubsegmentLengths(rangeXY_, bX_, cX_, dX_, bY_, cY_, dY_);
}

CubicSplineMT::CubicSplineMT(const std::vector<Voxel>& vs)
{
    auto [xs, ys] = Unzip(vs);

    rangeXY_ = linspace(xs.size(), 0., static_cast<double>(xs.size() - 1));
    std::tie(aX_, bX_, cX_, dX_) = FitSplineMT(rangeXY_, xs);
    std::tie(aY_, bY_, cY_, dY_) = FitSplineMT(rangeXY_, ys);
    std::tie(subsegLens_, cumuLens_) =
        SubsegmentLengths(rangeXY_, bX_, cX_, dX_, bY_, cY_, dY_);
}

// Evaluate the spline at a given value of t
auto CubicSplineMT::operator()(const double t) const -> Pixel
{
    assert(
        !cumuLens_.empty() &&
        "operator() called on a default-constructed CubicSplineMT");
    // Total length
    const auto totalLen = cumuLens_.back();
    const auto targetLen = totalLen * t;

    // Find the correct subsegment using binary search
    const auto it =
        std::lower_bound(cumuLens_.begin(), cumuLens_.end(), targetLen);
    std::size_t idx{0};
    if (it != cumuLens_.begin()) {
        idx = std::distance(cumuLens_.begin(), it) - 1;
    }

    const auto segCount = rangeXY_.size() - 1;
    const auto subsegCount = subsegLens_.size() / segCount;
    const auto segIdx = idx / subsegCount;
    const auto subsegIdx = idx % subsegCount;
    const auto subsegStart =
        static_cast<double>(subsegIdx) / static_cast<double>(subsegCount);
    const auto subsegEnd =
        static_cast<double>(subsegIdx + 1) / static_cast<double>(subsegCount);

    // Calculate the remaining length to target within this subsegment
    const auto remaining = targetLen - cumuLens_[idx];

    // Compute the x position at t
    const auto range0 = rangeXY_[segIdx];
    const auto range1 = rangeXY_[segIdx + 1];
    const auto rangeSub0 = range0 + (range1 - range0) * subsegStart;
    const auto rangeSub1 = range0 + (range1 - range0) * subsegEnd;
    const auto rangeT =
        rangeSub0 + (rangeSub1 - rangeSub0) * (remaining / subsegLens_[idx]);

    const double dRange = rangeT - range0;
    // Compute the x position at rangeT
    double xT = aX_[segIdx] + bX_[segIdx] * dRange +
                cX_[segIdx] * dRange * dRange +
                dX_[segIdx] * dRange * dRange * dRange;
    // Compute the y position at rangeT
    double yT = aY_[segIdx] + bY_[segIdx] * dRange +
                cY_[segIdx] * dRange * dRange +
                dY_[segIdx] * dRange * dRange * dRange;

    return {xT, yT};
}