#include <algorithm>
#include <cmath>
#include <iostream>
#include "fittedcurve.h"

using namespace volcart::segmentation;

double calcArcLength(const std::vector<Voxel>& vs);

FittedCurve::FittedCurve(const std::vector<Voxel>& vs, const int32_t zIndex)
    : npoints_(vs.size()), zIndex_(zIndex), seedPoints_(vs)
{
    std::vector<double> xs, ys;
    double arcLength = calcArcLength(vs);
    xs.reserve(vs.size());
    ys.reserve(vs.size());
    tvals_.reserve(vs.size());
    double accumulatedLength = 0;
    xs.push_back(vs.front()(0));
    ys.push_back(vs.front()(1));

    // Calculate new tvals_
    // Initial start t = 0
    tvals_.push_back(0);
    for (size_t i = 1; i < vs.size(); ++i) {
        xs.push_back(vs[i](0));
        ys.push_back(vs[i](1));
        accumulatedLength += std::sqrt(std::pow(vs[i](0) - vs[i - 1](0), 2) +
                                       std::pow(vs[i](1) - vs[i - 1](1), 2));
        tvals_.push_back(accumulatedLength / arcLength);
    }

    spline_ = CubicSpline<double>(xs, ys);

    // Calculate new voxel positions from the spline
    currentPoints_.reserve(vs.size());
    for (const auto t : tvals_) {
        auto p = spline_.eval(t);
        currentPoints_.emplace_back(p(0), p(1));
    }
}

std::vector<Voxel> FittedCurve::resample(const double resamplePerc)
{
    tvals_.clear();
    npoints_ = std::round(resamplePerc * npoints_);
    tvals_.resize(npoints_, 0.0);

    // Calculate new knot positions in t-space
    double sum = 0;
    for (int32_t i = 0; i < npoints_ && sum <= 1;
         ++i, sum += 1.0 / (npoints_ - 1)) {
        tvals_[i] = sum;
    }

    // Get new positions
    std::vector<Voxel> rs;
    rs.reserve(npoints_);
    currentPoints_.clear();
    for (const auto t : tvals_) {
        auto p = spline_.eval(t);
        currentPoints_.emplace_back(p(0), p(1));
        rs.emplace_back(p(0), p(1), zIndex_);
    }
    return rs;
}

Voxel FittedCurve::operator()(const int32_t index) const
{
    const auto t = tvals_[index];
    Pixel p = spline_.eval(t);
    return {p(0), p(1), double(zIndex_)};
}

Pixel FittedCurve::derivAt(const int32_t index, const int32_t hstep) const
{
    if (index - hstep <= 0) {
        return d1Forward(currentPoints_, index, hstep);
    } else if (index + hstep >= npoints_ - 1) {
        return d1Backward(currentPoints_, index, hstep);
    } else if (index - hstep < hstep || index + hstep >= npoints_ - hstep) {
        return d1Central(currentPoints_, index, hstep);
    } else {
        return d1FivePointStencil(currentPoints_, index, hstep);
    }
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

std::vector<double> FittedCurve::curvature(const int32_t hstep,
                                           const double scaleFactor) const
{
    std::vector<double> xs, ys;
    xs.reserve(currentPoints_.size());
    ys.reserve(currentPoints_.size());
    for (size_t t = 0; t < tvals_.size(); ++t) {
        xs.push_back(currentPoints_[t](0));
        ys.push_back(currentPoints_[t](1));
    }

    const auto dx1 = d1(xs, hstep);
    const auto dy1 = d1(ys, hstep);
    const auto dx2 = d2(xs, hstep);
    const auto dy2 = d2(ys, hstep);

    // Calculate curvature
    // according to: http://mathworld.wolfram.com/Curvature.html
    std::vector<double> k;
    k.reserve(currentPoints_.size());
    for (size_t i = 0; i < currentPoints_.size(); ++i) {
        k.push_back((dx1[i] * dy2[i] - dy1[i] * dx2[i]) /
                    std::pow(dx1[i] * dx1[i] + dy1[i] * dy1[i], 3.0 / 2.0) *
                    scaleFactor);
    }

    return k;
}
