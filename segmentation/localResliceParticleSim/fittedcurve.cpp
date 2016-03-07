#include <algorithm>
#include <cmath>
#include <iostream>
#include "fittedcurve.h"
#include "derivative.h"

using namespace volcart::segmentation;

double calcArcLength(const std::vector<Voxel>& vs);

std::vector<double> generateTVals(size_t count);

FittedCurve::FittedCurve(const std::vector<Voxel>& vs, int32_t zIndex)
    : npoints_(vs.size()), zIndex_(zIndex), ts_(generateTVals(npoints_))
{
    std::vector<double> xs, ys;
    std::tie(xs, ys) = unzip(vs);

    spline_ = CubicSpline<double>(xs, ys);

    // Calculate new voxel positions from the spline
    points_.reserve(vs.size());
    for (const auto t : ts_) {
        auto p = spline_.eval(t);
        points_.emplace_back(p(0), p(1), zIndex_);
    }
}

std::vector<Voxel> FittedCurve::resample(double resamplePerc)
{
    npoints_ = size_t(std::round(resamplePerc * npoints_));

    // If we're resampling at 100%, re-use last tvals
    if (resamplePerc != 1.0) {
        ts_ = generateTVals(npoints_);
    }

    // Get new voxel positions
    points_ = sample(npoints_);
    return points_;
}

std::vector<Voxel> FittedCurve::sample(size_t numPoints) const
{
    std::vector<Voxel> newPoints(numPoints);
    newPoints.reserve(numPoints);
    auto ts = generateTVals(numPoints);
    std::transform(std::begin(ts), std::end(ts), std::begin(newPoints),
                   [this](double t) -> Voxel {
                       auto p = spline_.eval(t);
                       return {p(0), p(1), zIndex_};
                   });
    return newPoints;
}

Voxel FittedCurve::operator()(int32_t index) const
{
    Pixel p = spline_.eval(ts_[index]);
    return {p(0), p(1), double(zIndex_)};
}

double calcArcLength(const std::vector<Voxel>& vs)
{
    double length = 0;
    for (size_t i = 1; i < vs.size(); ++i) {
        length += cv::norm(vs[i], vs[i - 1]);
    }
    return length;
}

std::vector<double> FittedCurve::curvature(int32_t hstep,
                                           const double scaleFactor) const
{
    std::vector<double> xs, ys;
    std::tie(xs, ys) = unzip(points_);
    const auto dx1 = d1(xs, hstep);
    const auto dy1 = d1(ys, hstep);
    const auto dx2 = d2(xs, hstep);
    const auto dy2 = d2(ys, hstep);

    // Calculate curvature
    // according to: http://mathworld.wolfram.com/Curvature.html
    std::vector<double> k;
    k.reserve(points_.size());
    for (size_t i = 0; i < points_.size(); ++i) {
        k.push_back((dx1[i] * dy2[i] - dy1[i] * dx2[i]) /
                    std::pow(dx1[i] * dx1[i] + dy1[i] * dy1[i], 3.0 / 2.0) *
                    scaleFactor);
    }

    return k;
}

std::vector<double> generateTVals(size_t count)
{
    std::vector<double> ts(count);
    ts[0] = 0;
    double sum = 0;
    std::generate(std::begin(ts) + 1, std::end(ts) - 1,
                  [count, &sum]() { return sum += 1.0 / (count - 1); });
    ts.back() = 1;
    return ts;
}
