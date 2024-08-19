/* This file is licensed under the MIT license. Please see CubicSplineMT.hpp. */

#include "vc/segmentation/lrps/CubicSplineMT.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gsl/gsl_integration.h>

#include "vc/core/util/Logging.hpp"

using namespace volcart;
using namespace volcart::segmentation;
using namespace Eigen;

using Index = Eigen::Index;

namespace
{

void Interpolate(
    const VectorXd& x,
    const VectorXd& y,
    VectorXd& a,
    VectorXd& b,
    VectorXd& c,
    VectorXd& d)
{
    const auto n = x.size() - 1;
    VectorXd h = x.segment(1, n) - x.segment(0, n);

    for (int i = 0; i < h.size(); ++i) {
        if (h(i) == 0)
            h(i) = 1e-8;
    }

    a = y.segment(0, n);
    b = VectorXd::Zero(n);
    c = VectorXd::Zero(n + 1);
    d = VectorXd::Zero(n);

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
}

void FitSplineWindow(
    const VectorXd& x,
    const VectorXd& y,
    std::vector<double>& aVec,
    std::vector<double>& bVec,
    std::vector<double>& cVec,
    std::vector<double>& dVec,
    const Index startIdx,
    const Index endIdx,
    const Index winSize,
    const Index bufSize,
    std::mutex& mtx)
{

    for (auto i = startIdx; i < endIdx; i += winSize) {
        const auto winEnd = std::min(i + winSize + bufSize, x.size());
        const auto winStart =
            std::max(winEnd - winSize - 2 * bufSize, Index{0});

        VectorXd xWin = x.segment(winStart, winEnd - winStart);
        VectorXd yWin = y.segment(winStart, winEnd - winStart);

        VectorXd a, b, c, d;
        Interpolate(xWin, yWin, a, b, c, d);

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

void FitSplineMT(
    const VectorXd& range,
    const VectorXd& val,
    VectorXd& a_total,
    VectorXd& b_total,
    VectorXd& c_total,
    VectorXd& d_total,
    std::mutex& mtx,
    const Index winSize = 100,
    const Index bufSize = 10,
    int numThreads = -1)
{
    // Init output params
    auto n = range.size();
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

    // TODO: Start here
    int steps = std::ceil(((double)n) / ((double)winSize));
    int steps_per_thread = steps / numThreads;
    int remainder = steps % numThreads;
    auto current_step = 0;
    for (auto i = 0; i < numThreads; ++i) {
        auto start_idx = current_step * winSize;
        auto end_idx = (current_step + steps_per_thread) * winSize;
        if (i < remainder) {
            end_idx += winSize;
            current_step += 1;
        }
        end_idx = std::min(end_idx, n);

        current_step += steps_per_thread;
        if (start_idx >= end_idx) {
            continue;
        }

        threads.emplace_back(
            &FitSplineWindow, std::ref(range), std::ref(val), std::ref(aVec),
            std::ref(bVec), std::ref(cVec), std::ref(dVec), start_idx, end_idx,
            winSize, bufSize, std::ref(mtx));
    }

    for (auto& t : threads) {
        t.join();
    }

    a_total = Eigen::Map<VectorXd>(aVec.data(), aVec.size());
    b_total = Eigen::Map<VectorXd>(bVec.data(), bVec.size());
    c_total = Eigen::Map<VectorXd>(cVec.data(), cVec.size());
    d_total = Eigen::Map<VectorXd>(dVec.data(), dVec.size());
}

double integrand2D(double t, void* params)
{
    double* coeffs = (double*)params;
    double b_x = coeffs[0], c_x = coeffs[1], d_x = coeffs[2];
    double b_y = coeffs[3], c_y = coeffs[4], d_y = coeffs[5];
    double t0 = coeffs[6];
    double ds_dx = b_x + 2 * c_x * (t - t0) + 3 * d_x * (t - t0) * (t - t0);
    double ds_dy = b_y + 2 * c_y * (t - t0) + 3 * d_y * (t - t0) * (t - t0);
    return sqrt(ds_dx * ds_dx + ds_dy * ds_dy);
}

double spline_length_2D(
    double b_x,
    double c_x,
    double d_x,
    double b_y,
    double c_y,
    double d_y,
    double t0,
    double t_sub0,
    double t_sub1)
{
    gsl_integration_workspace* w = gsl_integration_workspace_alloc(1000);
    double result, error;
    double coeffs[7] = {b_x, c_x, d_x, b_y, c_y, d_y, t0};
    gsl_function F;
    F.function = &integrand2D;
    F.params = &coeffs;
    gsl_integration_qags(&F, t_sub0, t_sub1, 0, 1e-8, 1000, w, &result, &error);
    gsl_integration_workspace_free(w);
    return fabs(result);
}

// Compute lengths of 2D spline segments
std::pair<VectorXd, VectorXd> compute_subsegment_lengths_2D(
    const VectorXd& t,
    const VectorXd& b_x,
    const VectorXd& c_x,
    const VectorXd& d_x,
    const VectorXd& b_y,
    const VectorXd& c_y,
    const VectorXd& d_y,
    int subseg_count = 10)
{
    std::vector<double> subsegment_lengths;
    std::vector<double> cumulative_lengths = {0.0};

    for (int i = 0; i < t.size() - 1; ++i) {
        double t0 = t[i], t1 = t[i + 1];
        for (int j = 0; j < subseg_count; ++j) {
            double t_sub0 =
                t0 + (t1 - t0) * (static_cast<double>(j) / subseg_count);
            double t_sub1 =
                t0 + (t1 - t0) * (static_cast<double>(j + 1) / subseg_count);
            double length = spline_length_2D(
                b_x[i], c_x[i], d_x[i], b_y[i], c_y[i], d_y[i], t0, t_sub0,
                t_sub1);
            subsegment_lengths.push_back(length);
            cumulative_lengths.push_back(cumulative_lengths.back() + length);
        }
    }
    return {
        Eigen::Map<VectorXd>(
            subsegment_lengths.data(), subsegment_lengths.size()),
        Eigen::Map<VectorXd>(
            cumulative_lengths.data(), cumulative_lengths.size())};
}

std::pair<double, double> evaluate_spline_at_t_2D(
    double t,
    const VectorXd& range_xy,
    const VectorXd& a_x,
    const VectorXd& b_x,
    const VectorXd& c_x,
    const VectorXd& d_x,
    const VectorXd& a_y,
    const VectorXd& b_y,
    const VectorXd& c_y,
    const VectorXd& d_y,
    const VectorXd& subsegment_lengths,
    const VectorXd& cumulative_lengths)
{

    double total_length = cumulative_lengths[cumulative_lengths.size() - 1];
    double target_length = total_length * t;

    // Find the correct subsegment using binary search
    auto it = std::lower_bound(
        cumulative_lengths.data(),
        cumulative_lengths.data() + cumulative_lengths.size(), target_length);
    int idx = std::distance(cumulative_lengths.data(), it) - 1;
    if (idx == -1) {
        idx = 0;
    }

    int seg_count = range_xy.size() - 1;
    int subseg_count = subsegment_lengths.size() / seg_count;
    int segment_idx = idx / subseg_count;
    int subseg_idx = idx % subseg_count;

    // Calculate the remaining length to target within this subsegment
    double remaining_length = target_length - cumulative_lengths[idx];

    // Compute the x position at t
    double range_xy0 = range_xy[segment_idx],
           range_xy1 = range_xy[segment_idx + 1];
    double range_xy_sub0 =
        range_xy0 + (range_xy1 - range_xy0) *
                        (static_cast<double>(subseg_idx) / subseg_count);
    double range_xy_sub1 =
        range_xy0 + (range_xy1 - range_xy0) *
                        (static_cast<double>(subseg_idx + 1) / subseg_count);
    double range_xy_t =
        range_xy_sub0 + (range_xy_sub1 - range_xy_sub0) *
                            (remaining_length / subsegment_lengths[idx]);

    double d_range_xy = range_xy_t - range_xy0;
    // Compute the x position at range_xy_t
    double x_t = a_x[segment_idx] + b_x[segment_idx] * d_range_xy +
                 c_x[segment_idx] * pow(d_range_xy, 2) +
                 d_x[segment_idx] * pow(d_range_xy, 3);
    // Compute the y position at range_xy_t
    double y_t = a_y[segment_idx] + b_y[segment_idx] * d_range_xy +
                 c_y[segment_idx] * pow(d_range_xy, 2) +
                 d_y[segment_idx] * pow(d_range_xy, 3);

    return {x_t, y_t};
}

}  // namespace

// Constructor implementation
CubicSplineMT::CubicSplineMT(const VectorXd& x, const VectorXd& y)
{
    range_xy_ =
        VectorXd::LinSpaced(x.size(), 0., static_cast<double>(x.size()) - 1.);
    npoints_ = x.size();
    FitSplineMT(range_xy_, x, a_x_, b_x_, c_x_, d_x_, mtx_);
    FitSplineMT(range_xy_, y, a_y_, b_y_, c_y_, d_y_, mtx_);
    std::tie(subsegment_lengths_, cumulative_lengths_) =
        compute_subsegment_lengths_2D(
            range_xy_, b_x_, c_x_, d_x_, b_y_, c_y_, d_y_);
}

CubicSplineMT::CubicSplineMT(const std::vector<Voxel>& vs)
{
    auto [xs, ys] = Unzip(vs);

    // Convert to VectorXd
    VectorXd xsEigen = VectorXd::Map(xs.data(), xs.size());
    VectorXd ysEigen = VectorXd::Map(ys.data(), ys.size());

    range_xy_ = VectorXd::LinSpaced((int)xsEigen.size(), 0, xsEigen.size() - 1);
    npoints_ = xs.size();
    FitSplineMT(range_xy_, xsEigen, a_x_, b_x_, c_x_, d_x_, mtx_);
    FitSplineMT(range_xy_, ysEigen, a_y_, b_y_, c_y_, d_y_, mtx_);
    std::tie(subsegment_lengths_, cumulative_lengths_) =
        compute_subsegment_lengths_2D(
            range_xy_, b_x_, c_x_, d_x_, b_y_, c_y_, d_y_);
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
auto CubicSplineMT::operator()(double t) const -> Pixel
{
    auto [x, y] = evaluate_spline_at_t_2D(
        t, range_xy_, a_x_, b_x_, c_x_, d_x_, a_y_, b_y_, c_y_, d_y_,
        subsegment_lengths_, cumulative_lengths_);
    return {x, y};
}