/* This file is licensed under the MIT license. Please see CubicSplineMT.hpp. */

#include "vc/segmentation/lrps/CubicSplineMT.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gsl/gsl_integration.h>

#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
using namespace volcart::segmentation;
using namespace Eigen;

using Index = Eigen::Index;
using Params = std::vector<double>;

namespace
{

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
        const auto winStart =
            std::max(winEnd - winSize - 2 * bufSize, std::size_t{0});
        const auto winStride = static_cast<Index>(winEnd - winStart);

        auto xWin = Eigen::Map<const VectorXd>(&x[winStart], winStride);
        auto yWin = Eigen::Map<const VectorXd>(&y[winStart], winStride);

        auto [a, b, c, d] = Interpolate(xWin, yWin);

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

    // TODO: There should be a thread pool for this
    std::vector<std::thread> threads;

    // Auto-determine the number of threads
    if (numThreads < 0) {
        numThreads = static_cast<int>(std::thread::hardware_concurrency());
        // TODO: Handle no threads
        assert(numThreads != 0);
    }
    Logger()->debug("Using {} threads", numThreads);
    threads.reserve(numThreads);

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
        threads.emplace_back(
            &FitSplineWindow, std::ref(range), std::ref(val), startIdx, endIdx,
            winSize, bufSize, std::ref(mtx), std::ref(aVec), std::ref(bVec),
            std::ref(cVec), std::ref(dVec));
    }

    // Wait for all threads to complete
    for (auto& t : threads) {
        t.join();
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
    double bX,
    double cX,
    double dX,
    double bY,
    double cY,
    double dY,
    double t0,
    double tSub0,
    double tSub1)
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
    const int nSegs = 10) -> std::tuple<Params, Params>
{
    const auto numLengths = nSegs * (t.size() - 1);
    Params subsegLengths;
    subsegLengths.reserve(numLengths);
    Params cumulativeLengths = {0.0};
    cumulativeLengths.reserve(numLengths + 1);

    for (const auto& [i, j] : range2D(t.size() - 1, nSegs)) {
        const auto t0 = t[i];
        const auto t1 = t[i + 1];
        const auto tSub0 = t0 + (t1 - t0) * (static_cast<double>(j) / nSegs);
        const auto tSub1 =
            t0 + (t1 - t0) * (static_cast<double>(j + 1) / nSegs);
        double length = SplineLength(
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
    range_xy_ = linspace(x.size(), 0., static_cast<double>(x.size() - 1));
    npoints_ = x.size();
    std::tie(a_x_, b_x_, c_x_, d_x_) = FitSplineMT(range_xy_, x, mtx_);
    std::tie(a_y_, b_y_, c_y_, d_y_) = FitSplineMT(range_xy_, y, mtx_);
    std::tie(subsegment_lengths_, cumulative_lengths_) =
        SubsegmentLengths(range_xy_, b_x_, c_x_, d_x_, b_y_, c_y_, d_y_);
}

CubicSplineMT::CubicSplineMT(const std::vector<Voxel>& vs)
{
    auto [xs, ys] = Unzip(vs);

    range_xy_ = linspace(xs.size(), 0., static_cast<double>(xs.size() - 1));
    npoints_ = xs.size();
    std::tie(a_x_, b_x_, c_x_, d_x_) = FitSplineMT(range_xy_, xs, mtx_);
    std::tie(a_y_, b_y_, c_y_, d_y_) = FitSplineMT(range_xy_, ys, mtx_);
    std::tie(subsegment_lengths_, cumulative_lengths_) =
        SubsegmentLengths(range_xy_, b_x_, c_x_, d_x_, b_y_, c_y_, d_y_);
}

CubicSplineMT::CubicSplineMT(const CubicSplineMT& other)
{
    a_x_ = other.a_x_;
    b_x_ = other.b_x_;
    c_x_ = other.c_x_;
    d_x_ = other.d_x_;
    a_y_ = other.a_y_;
    b_y_ = other.b_y_;
    c_y_ = other.c_y_;
    d_y_ = other.d_y_;
    range_xy_ = other.range_xy_;
    subsegment_lengths_ = other.subsegment_lengths_;
    cumulative_lengths_ = other.cumulative_lengths_;
    npoints_ = other.npoints_;

    // mtx is deliberately not copied
}

auto CubicSplineMT::operator=(const CubicSplineMT& other) -> CubicSplineMT&
{
    if (this != &other) {
        a_x_ = other.a_x_;
        b_x_ = other.b_x_;
        c_x_ = other.c_x_;
        d_x_ = other.d_x_;
        a_y_ = other.a_y_;
        b_y_ = other.b_y_;
        c_y_ = other.c_y_;
        d_y_ = other.d_y_;
        range_xy_ = other.range_xy_;
        subsegment_lengths_ = other.subsegment_lengths_;
        cumulative_lengths_ = other.cumulative_lengths_;
        npoints_ = other.npoints_;
        // mtx is deliberately not copied
    }

    return *this;
}

// Evaluate the spline at a given value of t
auto CubicSplineMT::operator()(const double t) const -> Pixel
{
    // Total length
    const auto totalLen = cumulative_lengths_.back();
    const auto targetLen = totalLen * t;

    // Find the correct subsegment using binary search
    const auto it = std::lower_bound(
        cumulative_lengths_.begin(), cumulative_lengths_.end(), targetLen);
    std::size_t idx{0};
    if (it != cumulative_lengths_.begin()) {
        idx = std::distance(cumulative_lengths_.begin(), it) - 1;
    }

    const auto seg_count = range_xy_.size() - 1;
    const auto subseg_count = subsegment_lengths_.size() / seg_count;
    const auto segment_idx = idx / subseg_count;
    const auto subseg_idx = idx % subseg_count;

    // Calculate the remaining length to target within this subsegment
    const auto remaining_length = targetLen - cumulative_lengths_[idx];

    // Compute the x position at t
    auto range_xy0 = range_xy_[segment_idx];
    auto range_xy1 = range_xy_[segment_idx + 1];
    const auto range_xy_sub0 =
        range_xy0 + (range_xy1 - range_xy0) *
                        (static_cast<double>(subseg_idx) / subseg_count);
    const auto range_xy_sub1 =
        range_xy0 + (range_xy1 - range_xy0) *
                        (static_cast<double>(subseg_idx + 1) / subseg_count);
    const auto range_xy_t =
        range_xy_sub0 + (range_xy_sub1 - range_xy_sub0) *
                            (remaining_length / subsegment_lengths_[idx]);

    const double d_range_xy = range_xy_t - range_xy0;
    // Compute the x position at range_xy_t
    double x_t = a_x_[segment_idx] + b_x_[segment_idx] * d_range_xy +
                 c_x_[segment_idx] * std::pow(d_range_xy, 2) +
                 d_x_[segment_idx] * std::pow(d_range_xy, 3);
    // Compute the y position at range_xy_t
    double y_t = a_y_[segment_idx] + b_y_[segment_idx] * d_range_xy +
                 c_y_[segment_idx] * pow(d_range_xy, 2) +
                 d_y_[segment_idx] * pow(d_range_xy, 3);

    return {x_t, y_t};
}