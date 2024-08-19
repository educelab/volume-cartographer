#pragma once

/** @file */

#include <mutex>
#include <vector>
#include <cstddef>

#include <Eigen/Dense>

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