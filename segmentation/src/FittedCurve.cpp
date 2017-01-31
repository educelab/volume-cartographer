/**@file FittedCurve.cpp */
#include <algorithm>
#include <cmath>
#include <iostream>

#include "segmentation/lrps/Derivative.hpp"
#include "segmentation/lrps/FittedCurve.hpp"

using namespace volcart::segmentation;

std::vector<double> GenerateTVals(size_t count);

FittedCurve::FittedCurve(const std::vector<Voxel>& vs, int zIndex)
    : npoints_(vs.size()), zIndex_(zIndex), ts_(GenerateTVals(npoints_))
{
    std::vector<double> xs, ys;
    std::tie(xs, ys) = Unzip(vs);

    spline_ = CubicSpline<double>(xs, ys);

    // Calculate new voxel positions from the spline
    points_.reserve(vs.size());
    for (const auto t : ts_) {
        auto p = spline_(t);
        points_.emplace_back(p(0), p(1), zIndex_);
    }
}

std::vector<Voxel> FittedCurve::resample(double resamplePerc)
{
    npoints_ = size_t(std::round(resamplePerc * npoints_));

    // If we're resampling at 100%, re-use last tvals
    if (resamplePerc != 1.0) {
        ts_ = GenerateTVals(npoints_);
    }

    // Get new voxel positions
    points_ = sample(npoints_);
    return points_;
}

std::vector<Voxel> FittedCurve::sample(size_t numPoints) const
{
    std::vector<Voxel> newPoints(numPoints);
    newPoints.reserve(numPoints);
    auto ts = GenerateTVals(numPoints);
    std::transform(
        std::begin(ts), std::end(ts), std::begin(newPoints),
        [this](auto t) -> Voxel {
            auto p = spline_(t);
            return {p(0), p(1), zIndex_};
        });
    return newPoints;
}

Voxel FittedCurve::operator()(int index) const
{
    assert(index >= 0 && index < int(ts_.size()) && "out of bounds");
    Pixel p = spline_(ts_[index]);
    return {p(0), p(1), double(zIndex_)};
}

std::vector<double> FittedCurve::curvature(int hstep) const
{
    std::vector<double> xs, ys;
    std::tie(xs, ys) = Unzip(points_);
    const auto dx1 = D1(xs, hstep);
    const auto dy1 = D1(ys, hstep);
    const auto dx2 = D2(xs, hstep);
    const auto dy2 = D2(ys, hstep);

    // Calculate curvature
    // according to: http://mathworld.wolfram.com/Curvature.html
    std::vector<double> k;
    k.reserve(points_.size());
    for (size_t i = 0; i < points_.size(); ++i) {
        k.push_back(
            (dx1[i] * dy2[i] - dy1[i] * dx2[i]) /
            std::pow(dx1[i] * dx1[i] + dy1[i] * dy1[i], 3.0 / 2.0));
    }

    return k;
}

double FittedCurve::arclength() const
{
    double length = 0;
    for (size_t i = 1; i < npoints_; ++i) {
        length += cv::norm(points_[i], points_[i - 1]);
    }
    return length;
}

std::vector<double> GenerateTVals(size_t count)
{
    std::vector<double> ts(count);
    if (count > 0) {
        ts[0] = 0;
        double sum = 0;
        std::generate(std::begin(ts) + 1, std::end(ts) - 1, [count, &sum]() {
            return sum += 1.0 / (count - 1);
        });
        ts.back() = 1;
    }
    return ts;
}
