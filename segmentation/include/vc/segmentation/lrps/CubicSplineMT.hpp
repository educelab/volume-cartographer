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
    std::vector<double> aX_, bX_, cX_, dX_;
    std::vector<double> aY_, bY_, cY_, dY_;
    std::vector<double> rangeXY_;
    std::vector<double> subsegLens_;
    std::vector<double> cumuLens_;
    std::mutex mtx_;
};