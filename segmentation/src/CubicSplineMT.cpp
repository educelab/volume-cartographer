/* This file is licensed under the MIT license. Please see CubicSplineMT.hpp. */

#include "vc/segmentation/lrps/CubicSplineMT.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <future>
#include <iostream>
#include <mutex>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <BS_thread_pool.hpp>
#include <gsl/gsl_integration.h>

#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
using namespace volcart::segmentation;
using namespace Eigen;

using Params = std::vector<double>;

namespace
{

// Compilation-unit local thread pool so threads are shared across runs
// TODO: At some point, this should become a singleton in vc::core.
BS::thread_pool POOL;

template <typename T>
auto linspace(const std::size_t num, const T low, const T high)
    -> std::vector<T>
{
    std::vector<T> v(num);
    auto step = (high - low) / static_cast<T>(num);
    std::generate(
        v.begin(), v.end(), [n = std::size_t{0}, &low, &step]() mutable {
            return low + step * n++;
        });
    return v;
}

auto Interpolate(const VectorXd& x, const VectorXd& y)
    -> std::tuple<VectorXd, VectorXd, VectorXd, VectorXd>
{
    // Result size
    const auto n = x.size() - 1;
    VectorXd h = x.segment(1, n) - x.segment(0, n);

    // Value must be non-zero
    for (int i = 0; i < h.size(); ++i) {
        if (h(i) == 0) {
            h(i) = 1e-8;
        }
    }

    // Result params
    const VectorXd a = y.segment(0, n);
    VectorXd b = VectorXd::Zero(n);
    VectorXd c = VectorXd::Zero(n + 1);
    VectorXd d = VectorXd::Zero(n);

    MatrixXd A = MatrixXd::Zero(n + 1, n + 1);
    VectorXd B = VectorXd::Zero(n + 1);

    A(0, 0) = 1;
    A(n, n) = 1;

    for (int i = 1; i < n; ++i) {
        A(i, i - 1) = h[i - 1];
        A(i, i) = 2 * (h[i - 1] + h[i]);
        A(i, i + 1) = h[i];
        B(i) = 3 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }

    c = A.colPivHouseholderQr().solve(B);

    for (int i = 0; i < n; ++i) {
        b(i) = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3;
        d(i) = (c[i + 1] - c[i]) / (3 * h[i]);
    }

    c.conservativeResize(n);

    return {a, b, c, d};
}

void FitSplineWindow(
    const Params& x,
    const Params& y,
    const std::size_t startIdx,
    const std::size_t endIdx,
    const std::size_t winSize,
    const std::size_t bufSize,
    std::mutex& mtx,
    Params& aVec,
    Params& bVec,
    Params& cVec,
    Params& dVec)
{
    for (auto i = startIdx; i < endIdx; i += winSize) {
        const auto winEnd = std::min(i + winSize + bufSize, x.size());
        std::size_t winStart{0};
        if (winEnd > winSize and winEnd - winSize > 2 * bufSize) {
            winStart = winEnd - winSize - 2 * bufSize;
        }
        const auto winStride = static_cast<Index>(winEnd - winStart);

        // Map vectors to Eigen vectors for interp
        auto xWin = Eigen::Map<const VectorXd>(&x[winStart], winStride);
        auto yWin = Eigen::Map<const VectorXd>(&y[winStart], winStride);

        // Interpolate
        auto [a, b, c, d] = Interpolate(xWin, yWin);

        // Copy results to vectors
        const auto updateStart = i - winStart;
        const auto updateEnd = std::min(i + winSize, x.size()) - winStart;
        std::lock_guard lock(mtx);
        std::copy(
            a.data() + updateStart, a.data() + updateEnd, aVec.begin() + i);
        std::copy(
            b.data() + updateStart, b.data() + updateEnd, bVec.begin() + i);
        std::copy(
            c.data() + updateStart, c.data() + updateEnd, cVec.begin() + i);
        std::copy(
            d.data() + updateStart, d.data() + updateEnd, dVec.begin() + i);
    }
}

auto FitSplineMT(
    const Params& range,
    const Params& val,
    std::mutex& mtx,
    const std::size_t winSize = 100,
    const std::size_t bufSize = 10,
    int numThreads = -1) -> std::tuple<Params, Params, Params, Params>
{
    // Init output params
    const auto n = range.size();
    std::vector aVec(n, 0.0);
    std::vector bVec(n, 0.0);
    std::vector cVec(n, 0.0);
    std::vector dVec(n, 0.0);

    // Thread futures
    std::vector<std::future<void>> futures;

    // Reset pool to the requested number of threads
    if (numThreads < 1 and
        POOL.get_thread_count() != std::thread::hardware_concurrency()) {
        POOL.reset();
    } else if (numThreads >= 1 and POOL.get_thread_count() != numThreads) {
        POOL.reset(numThreads);
    }
    numThreads = static_cast<int>(POOL.get_thread_count());
    Logger()->debug("Using {} threads", numThreads);
    futures.reserve(numThreads);

    // Fit spline windows on multiple threads
    const auto steps = static_cast<std::size_t>(
        std::ceil(static_cast<double>(n) / static_cast<double>(winSize)));
    const auto stepsPerThread = steps / numThreads;
    const auto remainder = steps % numThreads;
    std::size_t currentStep = 0;
    for (std::size_t i = 0; i < numThreads; ++i) {
        auto startIdx = currentStep * winSize;
        auto endIdx = (currentStep + stepsPerThread) * winSize;

        // Add one window from the remainder to every thread
        if (i < remainder) {
            endIdx += winSize;
            currentStep += 1;
        }
        endIdx = std::min(endIdx, n);

        // Update the step index
        currentStep += stepsPerThread;
        if (startIdx >= endIdx) {
            continue;
        }

        // Queue the job
        futures.emplace_back(
            POOL.submit_task([&range, &val, startIdx, endIdx, winSize, bufSize,
                              &mtx, &aVec, &bVec, &cVec, &dVec] {
                FitSplineWindow(
                    range, val, startIdx, endIdx, winSize, bufSize, mtx, aVec,
                    bVec, cVec, dVec);
            }));
    }

    // Wait for all threads to complete
    for (const auto& t : futures) {
        t.wait();
    }

    return {aVec, bVec, cVec, dVec};
}

double Integrand2D(const double t, void* params)
{
    const auto* coeffs = static_cast<double*>(params);
    const double bX = coeffs[0];
    const double cX = coeffs[1];
    const double dX = coeffs[2];
    const double bY = coeffs[3];
    const double cY = coeffs[4];
    const double dY = coeffs[5];
    const double t0 = coeffs[6];
    const double dsdx = bX + 2 * cX * (t - t0) + 3 * dX * (t - t0) * (t - t0);
    const double dsdy = bY + 2 * cY * (t - t0) + 3 * dY * (t - t0) * (t - t0);
    return std::sqrt(dsdx * dsdx + dsdy * dsdy);
}

double SplineLength(
    const double bX,
    const double cX,
    const double dX,
    const double bY,
    const double cY,
    const double dY,
    const double t0,
    const double tSub0,
    const double tSub1)
{
    auto* w = gsl_integration_workspace_alloc(1000);
    double result{0};
    double error{0};
    double coeffs[7] = {bX, cX, dX, bY, cY, dY, t0};
    gsl_function F;
    F.function = &Integrand2D;
    F.params = &coeffs;
    gsl_integration_qags(&F, tSub0, tSub1, 0, 1e-8, 1000, w, &result, &error);
    gsl_integration_workspace_free(w);
    return std::abs(result);
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
    std::tie(aX_, bX_, cX_, dX_) = FitSplineMT(rangeXY_, x, mtx_);
    std::tie(aY_, bY_, cY_, dY_) = FitSplineMT(rangeXY_, y, mtx_);
    std::tie(subsegLens_, cumuLens_) =
        SubsegmentLengths(rangeXY_, bX_, cX_, dX_, bY_, cY_, dY_);
}

CubicSplineMT::CubicSplineMT(const std::vector<Voxel>& vs)
{
    auto [xs, ys] = Unzip(vs);

    rangeXY_ = linspace(xs.size(), 0., static_cast<double>(xs.size() - 1));
    std::tie(aX_, bX_, cX_, dX_) = FitSplineMT(rangeXY_, xs, mtx_);
    std::tie(aY_, bY_, cY_, dY_) = FitSplineMT(rangeXY_, ys, mtx_);
    std::tie(subsegLens_, cumuLens_) =
        SubsegmentLengths(rangeXY_, bX_, cX_, dX_, bY_, cY_, dY_);
}

CubicSplineMT::CubicSplineMT(const CubicSplineMT& other)
{
    aX_ = other.aX_;
    bX_ = other.bX_;
    cX_ = other.cX_;
    dX_ = other.dX_;
    aY_ = other.aY_;
    bY_ = other.bY_;
    cY_ = other.cY_;
    dY_ = other.dY_;
    rangeXY_ = other.rangeXY_;
    subsegLens_ = other.subsegLens_;
    cumuLens_ = other.cumuLens_;

    // mtx is deliberately not copied
}

auto CubicSplineMT::operator=(const CubicSplineMT& other) -> CubicSplineMT&
{
    if (this != &other) {
        aX_ = other.aX_;
        bX_ = other.bX_;
        cX_ = other.cX_;
        dX_ = other.dX_;
        aY_ = other.aY_;
        bY_ = other.bY_;
        cY_ = other.cY_;
        dY_ = other.dY_;
        rangeXY_ = other.rangeXY_;
        subsegLens_ = other.subsegLens_;
        cumuLens_ = other.cumuLens_;
        // mtx is deliberately not copied
    }

    return *this;
}

// Evaluate the spline at a given value of t
auto CubicSplineMT::operator()(const double t) const -> Pixel
{
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
                cX_[segIdx] * std::pow(dRange, 2) +
                dX_[segIdx] * std::pow(dRange, 3);
    // Compute the y position at rangeT
    double yT = aY_[segIdx] + bY_[segIdx] * dRange +
                cY_[segIdx] * std::pow(dRange, 2) +
                dY_[segIdx] * std::pow(dRange, 3);

    return {xT, yT};
}