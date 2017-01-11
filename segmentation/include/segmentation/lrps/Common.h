#pragma once

#include <algorithm>
#include <tuple>
#include <vector>
#include <opencv2/core.hpp>

#define BGR_RED cv::Scalar(0, 0, 0xFF)
#define BGR_GREEN cv::Scalar(0, 0xFF, 0)
#define BGR_BLUE cv::Scalar(0xFF, 0, 0)
#define BGR_YELLOW cv::Scalar(0, 0xFF, 0xFF)
#define BGR_MAGENTA cv::Scalar(0xFF, 0, 0xFF)
#define BGR_BLACK cv::Scalar(0, 0, 0)

using IndexIntensityPair = std::pair<int, double>;
using IndexIntensityPairVec = typename std::vector<IndexIntensityPair>;
using Voxel = cv::Vec3d;
using Pixel = cv::Vec2d;

// Helpful for printing out vector. Only for debug.
template <typename T1, typename T2>
std::ostream& operator<<(std::ostream& s, std::pair<T1, T2> p)
{
    return s << "(" << std::get<0>(p) << ", " << std::get<1>(p) << ")";
}

template <typename T>
std::ostream& operator<<(std::ostream& s, std::vector<T> v)
{
    s << "[";
    if (v.size() == 0) {
        return s << "]";
    } else if (v.size() == 1) {
        return s << v[0] << "]";
    }
    // Need - 2 because v.end() points to one past the end of v.
    std::for_each(v.begin(), v.end() - 1, [&s](const T& t) { s << t << ", "; });
    return s << v.back() << "]";
}

namespace volcart
{
namespace segmentation
{

template <typename T1, typename T2>
std::vector<std::pair<T1, T2>> Zip(
    const std::vector<T1>& v1, const std::vector<T2>& v2)
{
    assert(v1.size() == v2.size() && "v1 and v2 must be the same size");
    std::vector<std::pair<T1, T2>> res;
    res.reserve(v1.size());
    for (int i = 0; i < int(v1.size()); ++i) {
        res.push_back(std::make_pair(v1[i], v2[i]));
    }
    return res;
}

template <typename T, int Length>
std::pair<std::vector<T>, std::vector<T>> Unzip(
    const std::vector<cv::Vec<T, Length>>& vs)
{
    std::vector<T> xs, ys;
    xs.reserve(vs.size());
    ys.reserve(vs.size());
    for (const auto& v : vs) {
        xs.push_back(v(0));
        ys.push_back(v(1));
    }
    return std::make_pair(xs, ys);
}

template <typename T>
std::vector<double> NormalizeVector(
    const std::vector<T>& v, double newMin = 0, double newMax = 1)
{
    // Check if values are already in desired range
    if (std::all_of(std::begin(v), std::end(v), [newMin, newMax](T e) {
            return newMin <= e && e <= newMax;
        })) {
        return std::vector<double>{std::begin(v), std::end(v)};
    }

    // Input checking
    if (v.empty()) {
        return std::vector<double>{};
    }
    if (v.size() == 1) {
        return std::vector<double>{newMax};
    }

    T min, max;
    auto p = std::minmax_element(begin(v), end(v));
    min = p->first;
    max = p->second;
    std::vector<double> vNorm(v.size());

    // Normalization of [min, max] --> [0, 1]
    std::transform(
        begin(v), end(v), begin(vNorm), [min, max, newMin, newMax](T t) {
            return ((newMax - newMin) / double(max - min)) * t +
                   ((newMin * max - min * newMax) / double(max - min));
        });
    return vNorm;
}

template <typename T, int Len>
std::vector<cv::Vec<double, Len>> NormalizeVector(
    const std::vector<cv::Vec<T, Len>> vs)
{
    std::vector<cv::Vec<double, Len>> vsNew(vs.size());
    std::transform(begin(vs), end(vs), std::begin(vsNew), [](auto v) {
        cv::Vec<double, Len> dv(v);
        if (cv::norm(dv) < 1e-5) {
            return dv;
        } else {
            return dv / cv::norm(dv);
        }
    });
    return vsNew;
}

// Some useful utility functions for doing math on std::vectors
std::vector<double> SquareDiff(
    const std::vector<Voxel>& v1, const std::vector<Voxel>& v2);

double SumSquareDiff(
    const std::vector<double>& v1, const std::vector<double>& v2);
}
}
