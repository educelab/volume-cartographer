/**
 * @file Common.hpp
 * @brief Commonly used typedefs and functions for LRPS
 * @ingroup lrps
 */
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

/**
 * @fn operator<<(std::ostream& s, std::pair<T1,T2> p)
 * @brief Write std::pair to std::ostream.
 *
 * Pair is formatted as round-bracketed, comma-separated elements. e.g.
 * "(T1, T2)"
 *
 * @param s Output stream
 * @param p Input pair
 */
template <typename T1, typename T2>
std::ostream& operator<<(std::ostream& s, std::pair<T1, T2> p)
{
    return s << "(" << std::get<0>(p) << ", " << std::get<1>(p) << ")";
}

/**
 * @fn operator<<(std::ostream& s, std::vector<T> v)
 * @brief Write std::vector to std::ostream.
 *
 * Vectors are formatted as square-bracketed, comma-separated elements.
 * e.g. "[a, b, c, d]"
 *
 * @param s Output stream
 * @param v Input vector
 */
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

/**
 * @fn std::vector< std::pair< T1, T2 > > zip(const std::vector<T1>& v1, const
 * std::vector<T2>& v2)
 * @brief Combine two equal-sized vectors into a single vector of paired
 * elements.
 *
 * Elements from v1 and v2 are combined into std::pair\<T1, T2\>.
 *
 * e.g. \n
 * v1 = [a, b, c, d] \n
 * v2 = [1, 2, 3, 4] \n
 * res = [\<a, 1\>, \<b, 2\>, \<c, 3\>, \<d, 4\>]
 *
 * @param v1 First vector to be combined
 * @param v2 Second vector to be combined
 */
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

/**
 * @fn std::pair<vector,vector> unzip(const std::vector<cv::Vec<T,Length>>&vs)
 * @brief Separate single vector of paired elements into two vectors
 * of single elements.
 *
 * @param vs Input vector
 */
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

/**
 * @fn std::vector<double> normalizeVector(const std::vector<T>& v, double
 *     newMin, double newMax)
 * @brief Normalize vector elements to within the range [newMin, newMax].
 *
 * @param v Vector to be normalized
 * @param newMin Minimum value
 * @param newMax Maximum value
 */
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

    auto p = std::minmax_element(begin(v), end(v));
    auto min = *p.first;
    auto max = *p.second;
    std::vector<double> vNorm(v.size());

    // Normalization of [min, max] --> [0, 1]
    std::transform(
        begin(v), end(v), begin(vNorm), [min, max, newMin, newMax](T t) {
            return ((newMax - newMin) / double(max - min)) * t +
                   ((newMin * max - min * newMax) / double(max - min));
        });
    return vNorm;
}

/**
 * @fn std::vector<int> normalizeVector(const
 *     std::vector<cv::Vec<T,Len>> vs)
 * @brief Normalizes a vector of cv::Vec using cv::norm
 *
 * @param vs Vector to be normalized
 */
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
/** @name VectorMath*/
//@{
/**
 * @fn std::vector<double> SquareDiff(const std::vector<Voxel>& v1, const
 *     std::vector<Voxel>& v2)
 * @brief Computes the difference of Squares on two vectors
 * @param v1 First vector to square
 * @param v2 Second vector to square
 */
std::vector<double> SquareDiff(
    const std::vector<Voxel>& v1, const std::vector<Voxel>& v2);

/**
 * @fn double SumSquareDiff(const std::vector<double>& v1, const
 *     std::vector<double>& v2)
 * @brief Sums the square differences between two vectors
 * @param v1 First vector to square
 * @param v2 Second vector to square
 */
double SumSquareDiff(
    const std::vector<double>& v1, const std::vector<double>& v2);
//@}
}
}
