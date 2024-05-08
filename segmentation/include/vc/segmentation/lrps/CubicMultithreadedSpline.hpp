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

#pragma once

#ifndef CUBICMULTITHREADEDSPLINE_H
#define CUBICMULTITHREADEDSPLINE_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <utility>
#include <numeric>
#include <thread>
#include <mutex>
#include "vc/segmentation/lrps/Common.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class CubicMultithreadedSpline {
private:
    VectorXd a_x_, b_x_, c_x_, d_x_;
    VectorXd a_y_, b_y_, c_y_, d_y_;
    VectorXd range_xy_;
    Eigen::VectorXd subsegment_lengths_;
    Eigen::VectorXd cumulative_lengths_;
    std::mutex mtx;
    int nr_points;

    void cubic_spline_interpolation(const VectorXd& x, const VectorXd& y,
                                    VectorXd& a, VectorXd& b,
                                    VectorXd& c, VectorXd& d);

    void windowed_spline_worker(const VectorXd& x, const VectorXd& y,
                                std::vector<double>& a_vec,
                                std::vector<double>& b_vec,
                                std::vector<double>& c_vec,
                                std::vector<double>& d_vec,
                                int start_idx, int end_idx,
                                int window_size, int buffer_size);

    void windowed_spline_with_buffer_multithreaded(const VectorXd& x, const VectorXd& y,
                                                VectorXd& a_total, VectorXd& b_total,
                                                VectorXd& c_total, VectorXd& d_total,
                                                int window_size = 100,
                                                int buffer_size = 10,
                                                int num_threads = -1);

static double integrand2D(double t, void *params);

double spline_length_2D(double b_x, double c_x, double d_x, double b_y, double c_y, double d_y, double t0, double t_sub0, double t_sub1);

// Compute lengths of 2D spline segments
std::pair<Eigen::VectorXd, Eigen::VectorXd> compute_subsegment_lengths_2D(
        const Eigen::VectorXd& t,
        const Eigen::VectorXd& b_x,
        const Eigen::VectorXd& c_x,
        const Eigen::VectorXd& d_x,
        const Eigen::VectorXd& b_y,
        const Eigen::VectorXd& c_y,
        const Eigen::VectorXd& d_y,
        int subseg_count = 10);

std::pair<double, double> evaluate_spline_at_t_2D(
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
        const Eigen::VectorXd& cumulative_lengths) const;

public:
    CubicMultithreadedSpline() = default;
    CubicMultithreadedSpline(const VectorXd& x, const VectorXd& y);
    CubicMultithreadedSpline(const std::vector<Voxel>& vs);
    ~CubicMultithreadedSpline();
    // Custom copy constructor
    CubicMultithreadedSpline(const CubicMultithreadedSpline& other) {
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
        nr_points = other.nr_points;
        
        // mtx is deliberately not copied
    }
    
    // Custom copy assignment operator
    CubicMultithreadedSpline& operator=(const CubicMultithreadedSpline& other) {
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
            nr_points = other.nr_points;
            
            // mtx is deliberately not copied
        }
        
        return *this;
    }
    // Evaluate the spline at a given value of t
    Pixel operator()(double t) const;
};

#endif