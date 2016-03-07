#include <algorithm>
#include <cmath>
#include <iostream>
#include "fittedcurve.h"
#include "derivative.h"

using namespace volcart::segmentation;

double calcArcLength(const std::vector<Voxel>& vs);

std::vector<Voxel> pixelVectorToVoxelVector(const std::vector<Pixel>& ps,
                                            int32_t zIndex);

FittedCurve::FittedCurve(const std::vector<Voxel>& vs, int32_t zIndex)
    : npoints_(vs.size()), zIndex_(zIndex)
{
    std::vector<double> xs, ys;
    xs.reserve(vs.size());
    ys.reserve(vs.size());
    xs.push_back(vs.front()(0));
    ys.push_back(vs.front()(1));

    double arcLength = calcArcLength(vs);

    // Calculate new ts_
    // Initial start t = 0
    double accumulatedLength = 0;
    ts_.reserve(vs.size());
    ts_.push_back(0);
    for (size_t i = 1; i < vs.size(); ++i) {
        xs.push_back(vs[i](0));
        ys.push_back(vs[i](1));
        accumulatedLength += std::sqrt(std::pow(vs[i](0) - vs[i - 1](0), 2) +
                                       std::pow(vs[i](1) - vs[i - 1](1), 2));
        ts_.push_back(accumulatedLength / arcLength);
    }

    spline_ = CubicSpline<double>(xs, ys);

    // Calculate new voxel positions from the spline
    points_.reserve(vs.size());
    for (const auto t : ts_) {
        auto p = spline_.eval(t);
        points_.emplace_back(p(0), p(1));
        xs_.push_back(p(0));
        ys_.push_back(p(1));
    }
}

std::vector<Voxel> FittedCurve::resample(double resamplePerc)
{
    // If we're resampling at 100%, then just use the work we did in the
    // constructor
    if (resamplePerc == 1.0) {
        return pixelVectorToVoxelVector(points_, zIndex_);
    }

    ts_.clear();
    xs_.clear();
    ys_.clear();
    npoints_ = std::round(resamplePerc * npoints_);
    ts_.resize(npoints_, 0.0);

    // Calculate new knot positions in t-space
    double sum = 0;
    for (int32_t i = 0; i < npoints_ && sum <= 1;
         ++i, sum += 1.0 / (npoints_ - 1)) {
        ts_[i] = sum;
    }
    ts_.back() = 1.0;

    // Get new positions
    std::vector<Voxel> rs;
    rs.reserve(npoints_);
    points_.clear();
    for (const auto t : ts_) {
        auto p = spline_.eval(t);
        points_.emplace_back(p(0), p(1));
        xs_.push_back(p(0));
        ys_.push_back(p(1));
        rs.emplace_back(p(0), p(1), zIndex_);
    }
    return rs;
}

std::vector<Voxel> FittedCurve::resample(int32_t numPoints) const
{
    std::vector<Voxel> newPoints;
    newPoints.reserve(numPoints);
    double sum = 0;
    for (int32_t i = 0; i < numPoints && sum <= 1;
         ++i, sum += 1.0 / (numPoints - 1)) {
        auto p = spline_.eval(sum);
        newPoints.emplace_back(p(0), p(1), zIndex_);
    }
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
        length += std::sqrt(std::pow(vs[i](0) - vs[i - 1](0), 2) +
                            std::pow(vs[i](1) - vs[i - 1](1), 2));
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

// Helper function to convert a vector of Pixel to a vector of Voxel
std::vector<Voxel> pixelVectorToVoxelVector(const std::vector<Pixel>& ps,
                                            int32_t zIndex)
{
    std::vector<Voxel> old_vs;
    old_vs.reserve(ps.size());
    std::transform(std::begin(ps), std::end(ps), std::back_inserter(old_vs),
                   [zIndex](const Pixel p) -> Voxel {
                       return {p(0), p(1), zIndex};
                   });
    return old_vs;
}
