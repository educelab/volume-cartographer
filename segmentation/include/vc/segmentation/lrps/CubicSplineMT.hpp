#pragma once

/** @file */

#include <mutex>
#include <vector>
#include <cstddef>

#include "vc/segmentation/lrps/Common.hpp"

/**
 * @brief Thread-safe cubic spline
 *
 * @author    Julian Schilliger
 * @date      September 2023
 * @copyright 2023 Julian Schilliger, MIT License.
 */
class CubicSplineMT
{
public:
    CubicSplineMT() = default;
    CubicSplineMT(const std::vector<double>&(x), const std::vector<double>&(y));
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
    std::vector<double> a_x_, b_x_, c_x_, d_x_;
    std::vector<double> a_y_, b_y_, c_y_, d_y_;
    std::vector<double> range_xy_;
    std::vector<double> subsegment_lengths_;
    std::vector<double> cumulative_lengths_;
    std::mutex mtx_;
    std::size_t npoints_{0};
};