/*
 * File: CubicMultithreadedSpline.cpp
 * Author: Julian Schilliger
 * 
 * Created on: September 2023
 *
 * Description: CubicMultithreadedSpline class implementation for scalar spline interpolation.
 *
 * License: MIT License
 *
 * Copyright (c) 2023 Julian Schilliger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <utility>
#include <numeric>
#include <thread>
#include <mutex>

#include <gsl/gsl_integration.h>


#include "vc/segmentation/lrps/CubicMultithreadedSpline.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace volcart::segmentation;

// Constructor implementation
CubicMultithreadedSpline::CubicMultithreadedSpline(const VectorXd& x, const VectorXd& y)
{
    range_xy_ = Eigen::VectorXd::LinSpaced((int)x.size(), 0, x.size()-1);
    nr_points = x.size();
    windowed_spline_with_buffer_multithreaded(range_xy_, x, a_x_, b_x_, c_x_, d_x_);
    windowed_spline_with_buffer_multithreaded(range_xy_, y, a_y_, b_y_, c_y_, d_y_);
    std::tie(subsegment_lengths_, cumulative_lengths_) = compute_subsegment_lengths_2D(range_xy_, b_x_, c_x_, d_x_, b_y_, c_y_, d_y_);
}

CubicMultithreadedSpline::CubicMultithreadedSpline(const std::vector<Voxel>& vs) {
    std::vector<double> xs, ys;
    std::tie(xs, ys) = Unzip(vs);

    // Convert to Eigen::VectorXd
    Eigen::VectorXd xsEigen = Eigen::VectorXd::Map(xs.data(), xs.size());
    Eigen::VectorXd ysEigen = Eigen::VectorXd::Map(ys.data(), ys.size());

    range_xy_ = Eigen::VectorXd::LinSpaced((int)xsEigen.size(), 0, xsEigen.size()-1);
    nr_points = xs.size();
    windowed_spline_with_buffer_multithreaded(range_xy_, xsEigen, a_x_, b_x_, c_x_, d_x_);
    windowed_spline_with_buffer_multithreaded(range_xy_, ysEigen, a_y_, b_y_, c_y_, d_y_);
    std::tie(subsegment_lengths_, cumulative_lengths_) = compute_subsegment_lengths_2D(range_xy_, b_x_, c_x_, d_x_, b_y_, c_y_, d_y_);
}

CubicMultithreadedSpline::~CubicMultithreadedSpline() {}

// Evaluate the spline at a given value of t
Pixel CubicMultithreadedSpline::operator()(double t) const {
    auto [x, y] = evaluate_spline_at_t_2D(t, range_xy_, a_x_, b_x_, c_x_, d_x_, a_y_, b_y_, c_y_, d_y_, subsegment_lengths_, cumulative_lengths_);
    return Pixel(x, y); 
}

void CubicMultithreadedSpline::cubic_spline_interpolation(const VectorXd& x, const VectorXd& y,
                                VectorXd& a, VectorXd& b,
                                VectorXd& c, VectorXd& d) {
    int n = x.size() - 1;
    VectorXd h = x.segment(1, n) - x.segment(0, n);

    for(int i = 0; i < h.size(); ++i) {
        if(h(i) == 0) h(i) = 1e-8;
    }

    a = y.segment(0, n);
    b = VectorXd::Zero(n);
    c = VectorXd::Zero(n + 1);
    d = VectorXd::Zero(n);

    MatrixXd A = MatrixXd::Zero(n + 1, n + 1);
    VectorXd B = VectorXd::Zero(n + 1);

    A(0, 0) = 1;
    A(n, n) = 1;

    for(int i = 1; i < n; ++i) {
        A(i, i - 1) = h[i - 1];
        A(i, i) = 2 * (h[i - 1] + h[i]);
        A(i, i + 1) = h[i];
        B(i) = 3 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }

    c = A.colPivHouseholderQr().solve(B);

    for(int i = 0; i < n; ++i) {
        b(i) = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3;
        d(i) = (c[i + 1] - c[i]) / (3 * h[i]);
    }

    c.conservativeResize(n);
}

void CubicMultithreadedSpline::windowed_spline_worker(const VectorXd& x, const VectorXd& y,
                            std::vector<double>& a_vec,
                            std::vector<double>& b_vec,
                            std::vector<double>& c_vec,
                            std::vector<double>& d_vec,
                            int start_idx, int end_idx,
                            int window_size, int buffer_size) {

    for (int i = start_idx; i < end_idx; i += window_size) {
        int wnd_end_idx = std::min(i + window_size + buffer_size, (int)x.size());
        int wnd_start_idx = std::max(wnd_end_idx - window_size - 2 * buffer_size, 0);

        VectorXd x_window = x.segment(wnd_start_idx, wnd_end_idx - wnd_start_idx);
        VectorXd y_window = y.segment(wnd_start_idx, wnd_end_idx - wnd_start_idx);

        VectorXd a, b, c, d;
        cubic_spline_interpolation(x_window, y_window, a, b, c, d);

        int idx_start = i - wnd_start_idx;
        int idx_end = std::min(i + window_size, (int)x.size()) - wnd_start_idx;

        std::lock_guard<std::mutex> lock(mtx);
        std::copy(a.data() + idx_start, a.data() + idx_end, a_vec.begin() + i);
        std::copy(b.data() + idx_start, b.data() + idx_end, b_vec.begin() + i);
        std::copy(c.data() + idx_start, c.data() + idx_end, c_vec.begin() + i);
        std::copy(d.data() + idx_start, d.data() + idx_end, d_vec.begin() + i);
    }
}

void CubicMultithreadedSpline::windowed_spline_with_buffer_multithreaded(const VectorXd& x, const VectorXd& y,
                                            VectorXd& a_total, VectorXd& b_total,
                                            VectorXd& c_total, VectorXd& d_total,
                                            int window_size,
                                            int buffer_size,
                                            int num_threads) {

    int n = x.size();
    std::vector<double> a_vec(n, 0.0), b_vec(n, 0.0), c_vec(n, 0.0), d_vec(n, 0.0);

    std::vector<std::thread> threads;

    if (num_threads == -1) {
        int num_available_threads = static_cast<int>(std::thread::hardware_concurrency());
        num_threads = num_available_threads;
    }

    int steps = std::ceil(((double)n) / ((double)window_size));
    int steps_per_thread = steps / num_threads;
    int remainder = steps % num_threads;
    int current_step = 0;
    for (int i = 0; i < num_threads; ++i) {
        int start_idx = current_step * window_size;
        int end_idx = (current_step + steps_per_thread) * window_size;
        if (i < remainder) {
            end_idx += window_size;
            current_step += 1;
        }
        end_idx = std::min(end_idx, n);
        
        current_step += steps_per_thread;
        if (start_idx >= end_idx) {
            continue;
        }

        threads.emplace_back(std::thread(&CubicMultithreadedSpline::windowed_spline_worker, this, std::ref(x), std::ref(y),
                                        std::ref(a_vec), std::ref(b_vec), std::ref(c_vec),
                                        std::ref(d_vec), start_idx, end_idx,
                                        window_size, buffer_size));
    }

    for (auto& t : threads) {
        t.join();
    }

    a_total = Eigen::Map<VectorXd>(a_vec.data(), a_vec.size());
    b_total = Eigen::Map<VectorXd>(b_vec.data(), b_vec.size());
    c_total = Eigen::Map<VectorXd>(c_vec.data(), c_vec.size());
    d_total = Eigen::Map<VectorXd>(d_vec.data(), d_vec.size());
}

double CubicMultithreadedSpline::integrand2D(double t, void *params) {
    double *coeffs = (double *)params;
    double b_x = coeffs[0], c_x = coeffs[1], d_x = coeffs[2];
    double b_y = coeffs[3], c_y = coeffs[4], d_y = coeffs[5];
    double t0 = coeffs[6];
    double ds_dx = b_x + 2 * c_x * (t - t0) + 3 * d_x * (t - t0) * (t - t0);
    double ds_dy = b_y + 2 * c_y * (t - t0) + 3 * d_y * (t - t0) * (t - t0);
    return sqrt(ds_dx * ds_dx + ds_dy * ds_dy);
}

double CubicMultithreadedSpline::spline_length_2D(double b_x, double c_x, double d_x, double b_y, double c_y, double d_y, double t0, double t_sub0, double t_sub1) {
    gsl_integration_workspace *w = gsl_integration_workspace_alloc(1000);
    double result, error;
    double coeffs[7] = {b_x, c_x, d_x, b_y, c_y, d_y, t0};
    gsl_function F;
    F.function = &CubicMultithreadedSpline::integrand2D;
    F.params = &coeffs;
    gsl_integration_qags(&F, t_sub0, t_sub1, 0, 1e-8, 1000, w, &result, &error);
    gsl_integration_workspace_free(w);
    return fabs(result);
}

// Compute lengths of 2D spline segments
std::pair<Eigen::VectorXd, Eigen::VectorXd> CubicMultithreadedSpline::compute_subsegment_lengths_2D(
        const Eigen::VectorXd& t,
        const Eigen::VectorXd& b_x,
        const Eigen::VectorXd& c_x,
        const Eigen::VectorXd& d_x,
        const Eigen::VectorXd& b_y,
        const Eigen::VectorXd& c_y,
        const Eigen::VectorXd& d_y,
        int subseg_count) {
    std::vector<double> subsegment_lengths;
    std::vector<double> cumulative_lengths = {0.0};
    
    for (int i = 0; i < t.size() - 1; ++i) {
        double t0 = t[i], t1 = t[i + 1];
        for (int j = 0; j < subseg_count; ++j) {
            double t_sub0 = t0 + (t1 - t0) * (static_cast<double>(j) / subseg_count);
            double t_sub1 = t0 + (t1 - t0) * (static_cast<double>(j + 1) / subseg_count);
            double length = spline_length_2D(b_x[i], c_x[i], d_x[i], b_y[i], c_y[i], d_y[i], t0, t_sub0, t_sub1);
            subsegment_lengths.push_back(length);
            cumulative_lengths.push_back(cumulative_lengths.back() + length);
        }
    }
    return {Eigen::Map<Eigen::VectorXd>(subsegment_lengths.data(), subsegment_lengths.size()),
            Eigen::Map<Eigen::VectorXd>(cumulative_lengths.data(), cumulative_lengths.size())};
}

std::pair<double, double> CubicMultithreadedSpline::evaluate_spline_at_t_2D(
        double t,
        const Eigen::VectorXd& range_xy,
        const Eigen::VectorXd& a_x,
        const Eigen::VectorXd& b_x,
        const Eigen::VectorXd& c_x,
        const Eigen::VectorXd& d_x,
        const Eigen::VectorXd& a_y,
        const Eigen::VectorXd& b_y,
        const Eigen::VectorXd& c_y,
        const Eigen::VectorXd& d_y,
        const Eigen::VectorXd& subsegment_lengths,
        const Eigen::VectorXd& cumulative_lengths) const {

    double total_length = cumulative_lengths[cumulative_lengths.size() - 1];
    double target_length = total_length * t;
    
    // Find the correct subsegment using binary search
    auto it = std::lower_bound(cumulative_lengths.data(), cumulative_lengths.data() + cumulative_lengths.size(), target_length);
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
    double range_xy0 = range_xy[segment_idx], range_xy1 = range_xy[segment_idx + 1];
    double range_xy_sub0 = range_xy0 + (range_xy1 - range_xy0) * (static_cast<double>(subseg_idx) / subseg_count);
    double range_xy_sub1 = range_xy0 + (range_xy1 - range_xy0) * (static_cast<double>(subseg_idx + 1) / subseg_count);
    double range_xy_t = range_xy_sub0 + (range_xy_sub1 - range_xy_sub0) * (remaining_length / subsegment_lengths[idx]);
    
    double d_range_xy = range_xy_t - range_xy0;
    // Compute the x position at range_xy_t
    double x_t = a_x[segment_idx] + b_x[segment_idx]*d_range_xy + c_x[segment_idx]*pow(d_range_xy, 2) + d_x[segment_idx]*pow(d_range_xy, 3);
    // Compute the y position at range_xy_t
    double y_t = a_y[segment_idx] + b_y[segment_idx]*d_range_xy + c_y[segment_idx]*pow(d_range_xy, 2) + d_y[segment_idx]*pow(d_range_xy, 3);

    
    return {x_t, y_t};
}