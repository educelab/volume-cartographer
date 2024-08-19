/*
 * File: CubicMultithreadedSpline.cpp
 * Author: Julian Schilliger
 *
 * Created on: September 2023
 *
 * Description: CubicMultithreadedSpline class implementation for scalar spline
 * interpolation.
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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

/** @file */

#include <mutex>
#include <vector>
#include <cstddef>

#include <Eigen/Dense>

#include "vc/segmentation/lrps/Common.hpp"

/** @brief Thread-safe cubic spline */
class CubicSplineMT
{
public:
    CubicSplineMT() = default;
    CubicSplineMT(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
    explicit CubicSplineMT(const std::vector<Voxel>& vs);
    ~CubicSplineMT() = default;

    /** Copy constructor */
    CubicSplineMT(const CubicSplineMT& other);

    /** Custom copy assignment operator */
    auto operator=(const CubicSplineMT& other) -> CubicSplineMT&;

    /**
     * @brief %Spline evaluation at t-space value t in [0, 1]
     */
    auto operator()(double t) const -> Pixel;

private:
    Eigen::VectorXd a_x_, b_x_, c_x_, d_x_;
    Eigen::VectorXd a_y_, b_y_, c_y_, d_y_;
    Eigen::VectorXd range_xy_;
    Eigen::VectorXd subsegment_lengths_;
    Eigen::VectorXd cumulative_lengths_;
    std::mutex mtx_;
    std::size_t npoints_{0};
};