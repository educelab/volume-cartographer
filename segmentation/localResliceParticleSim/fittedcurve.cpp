#include <algorithm>
#include <cmath>
#include <iostream>
#include "fittedcurve.h"

using namespace volcart::segmentation;

double calcArcLength(const vec<Voxel>& vs);

FittedCurve::FittedCurve(const vec<Voxel>& vs, const int32_t zIndex)
    : npoints_(vs.size()), zIndex_(zIndex), seedPoints_(vs)
{
    vec<double> xs, ys;
    double arcLength = calcArcLength(vs);
    xs.reserve(vs.size());
    ys.reserve(vs.size());
    tvals.reserve(vs.size());
    double accumulatedLength = 0;
    xs.push_back(vs.front()(0));
    ys.push_back(vs.front()(1));

    // Calculate new tvals
    // Initial start t = 0
    tvals.push_back(0);
    for (size_t i = 1; i < vs.size(); ++i) {
        xs.push_back(vs[i](0));
        ys.push_back(vs[i](1));
        accumulatedLength += std::sqrt(std::pow(vs[i](0) - vs[i - 1](0), 2) +
                                       std::pow(vs[i](1) - vs[i - 1](1), 2));
        tvals.push_back(accumulatedLength / arcLength);
    }

    spline_ = CubicSpline<double>(xs, ys);
}

vec<Voxel> FittedCurve::resample(const double resamplePerc)
{
    tvals.clear();
    npoints_ = std::round(resamplePerc * npoints_);
    tvals.resize(npoints_, 0.0);

    // Calculate new knot positions in t-space
    double sum = 0;
    for (int32_t i = 0; i < npoints_ && sum <= 1;
         ++i, sum += 1.0 / (npoints_ - 1)) {
        tvals[i] = sum;
    }

    // Get new positions
    vec<Voxel> rs;
    rs.reserve(npoints_);
    std::transform(tvals.begin(), tvals.end(), std::back_inserter(rs),
                   [this](double t) -> Voxel {
                       auto p = spline_.eval(t);
                       return {p(0), p(1), double(zIndex_)};
                   });
    return rs;
}

Voxel FittedCurve::operator()(const int32_t index) const
{
    const auto t = tvals[index];
    Pixel p = spline_.eval(t);
    return {p(0), p(1), double(zIndex_)};
}

Voxel FittedCurve::derivAt(const int32_t index, const int32_t hstep) const
{
    if (index - hstep <= 0) {
        return derivForwardDifference(index, hstep);
    } else if (index + hstep >= npoints_ - 1) {
        return derivBackwardDifference(index, hstep);
    } else if (index - hstep < hstep || index + hstep >= npoints_ - hstep) {
        return derivCentralDifference(index, hstep);
    } else {
        return derivFivePointStencil(index, hstep);
    }
}

vec<double> FittedCurve::deriv(const int32_t hstep) const
{
    vec<double> ps;
    ps.reserve(npoints_);
    for (int32_t i = 0; i < npoints_; ++i) {
        ps.push_back(derivAt(i, hstep)(1));
    }
    return ps;
}

Voxel FittedCurve::derivCentralDifference(const int32_t index,
                                          const int32_t hstep) const
{
    assert(index >= 1 && index <= int32_t(tvals.size() - 1) &&
           "index must not be an endpoint\n");
    double before = tvals[index - hstep];
    double after = tvals[index + hstep];
    auto p = spline_.eval(after) - spline_.eval(before);
    return {p(0), p(1), double(zIndex_)};
}

Voxel FittedCurve::derivBackwardDifference(const int32_t index,
                                           const int32_t hstep) const
{
    assert(index >= 1 && "index must not be first point\n");
    double current = tvals[index];
    double before = tvals[index - hstep];
    auto p = spline_.eval(current) - spline_.eval(before);
    return {p(0), p(1), double(zIndex_)};
}

Voxel FittedCurve::derivForwardDifference(const int32_t index,
                                          const int32_t hstep) const
{
    assert(index <= int32_t(tvals.size() - 2) &&
           "index must not be last point\n");
    double current = tvals[index];
    double after = tvals[index + hstep];
    auto p = spline_.eval(after) - spline_.eval(current);
    return {p(0), p(1), double(zIndex_)};
}

Voxel FittedCurve::derivFivePointStencil(const int32_t index,
                                         const int32_t hstep) const
{
    assert(index >= 2 * hstep && index <= int32_t(tvals.size() - 2 * hstep) &&
           "index must not be first/last two points for consideration\n");
    double before2 = tvals[index - (2 * hstep)];
    double before1 = tvals[index - hstep];
    double after1 = tvals[index + hstep];
    double after2 = tvals[index + (2 * hstep)];
    auto p = -spline_.eval(after2) + 8 * spline_.eval(after1) -
             8 * spline_.eval(before1) + spline_.eval(before2);
    return {p(0), p(1), double(zIndex_)};
}

double calcArcLength(const vec<Voxel>& vs)
{
    double length = 0;
    for (size_t i = 1; i < vs.size(); ++i) {
        length += std::sqrt(std::pow(vs[i](0) - vs[i - 1](0), 2) +
                            std::pow(vs[i](1) - vs[i - 1](1), 2));
    }
    return length;
}
