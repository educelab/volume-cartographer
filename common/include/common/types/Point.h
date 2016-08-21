#pragma once

#include "common/util/zip.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <sstream>
#include <tuple>
#include <type_traits>

namespace volcart
{
template <typename T,
          size_t N,
          typename =
              typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
class Point
{
public:
    using Element = T;
    using Container = std::array<T, N>;
    using Iterator = typename Container::iterator;
    using ConstIterator = typename Container::const_iterator;
    static constexpr size_t dim = N;

    Point() = default;

    template <typename... Ts>
    Point(Ts... args)
    {
        static_assert(sizeof...(args) == N, "Not the right # args");
        data_ = {args...};
    }

    Point(std::initializer_list<T> vals)
    {
        std::copy(std::begin(vals), std::end(vals), std::begin(data_));
    }

    static Point fill(T fillVal)
    {
        Point p;
        std::fill(std::begin(p), std::end(p), fillVal);
        return p;
    }

    Point& operator+=(const Point& p)
    {
        for (size_t i = 0; i < p.dim; ++i) {
            data_[i] += p[i];
        }
        return *this;
    }

    Point& operator-=(const Point& p)
    {
        for (size_t i = 0; i < p.dim; ++i) {
            data_[i] -= p[i];
        }
        return *this;
    }

    template <typename Scalar,
              typename = typename std::
                  enable_if<std::is_arithmetic<Scalar>::value, Scalar>::type>
    Point& operator+=(Scalar s)
    {
        for (size_t i = 0; i < dim; ++i) {
            data_[i] += s;
        }
        return *this;
    }

    template <typename Scalar,
              typename = typename std::
                  enable_if<std::is_arithmetic<Scalar>::value, Scalar>::type>
    Point& operator-=(Scalar s)
    {
        for (size_t i = 0; i < dim; ++i) {
            data_[i] -= s;
        }
        return *this;
    }

    template <typename Scalar,
              typename = typename std::
                  enable_if<std::is_arithmetic<Scalar>::value, Scalar>::type>
    Point& operator*=(Scalar s)
    {
        for (size_t i = 0; i < dim; ++i) {
            data_[i] *= s;
        }
        return *this;
    }

    template <typename Scalar,
              typename = typename std::
                  enable_if<std::is_arithmetic<Scalar>::value, Scalar>::type>
    Point& operator/=(Scalar s)
    {
        for (size_t i = 0; i < dim; ++i) {
            data_[i] /= s;
        }
        return *this;
    }

    double norm() const
    {
        std::array<T, N> squared;
        std::transform(std::begin(data_), std::end(data_), std::begin(squared),
                       [](T t) { return t * t; });
        auto sum = std::accumulate(std::begin(squared), std::end(squared), T{});
        return std::sqrt(sum);
    }

    // Iterators
    Iterator begin() { return std::begin(data_); }
    ConstIterator begin() const { return std::begin(data_); }
    Iterator end() { return std::end(data_); }
    ConstIterator end() const { return std::end(data_); }

    T& front() { return data_.front(); }
    const T& front() const { return data_.front(); }
    T& back() { return data_.back(); }
    const T& back() const { return data_.back(); }

    // Serialize to bytes
    char* bytes() { return reinterpret_cast<char*>(data_.data()); }
    const char* bytes() const
    {
        return reinterpret_cast<const char*>(data_.data());
    }

    // Accessors
    const T& operator[](size_t idx) const { return data_[idx]; }
    T& operator[](size_t idx) { return data_[idx]; }

    size_t size() const { return dim; }

    // Pretty print a Point (for logging, etc)
    std::string pprint() const
    {
        if (dim == 0) {
            return "[]";
        }
        std::stringstream ss;
        ss << "[";
        for (auto it = std::begin(data_); it != std::end(data_) - 1; ++it) {
            ss << std::to_string(*it) << ", ";
        }
        ss << std::to_string(back()) << "]";
        return ss.str();
    }

    cv::Vec<T, N> toCvVec() const { cv::Vec<T, N>(data_.data()); }

private:
    std::array<T, N> data_;
};

template <typename T>
using Point3 = Point<T, 3>;
using Point3d = Point3<double>;
using Point3f = Point3<float>;
using Point3i = Point3<int32_t>;
using Point3b = Point3<uint8_t>;

template <typename T>
using Point6 = Point<T, 6>;
using Point6d = Point6<double>;
using Point6f = Point6<float>;
using Point6i = Point6<int32_t>;
using Point6b = Point6<uint8_t>;

// Operations on points
template <typename T, size_t N>
Point<T, N> operator+(Point<T, N> lhs, const Point<T, N>& rhs)
{
    return lhs += rhs;
}

template <typename T, size_t N>
Point<T, N> operator-(Point<T, N> lhs, const Point<T, N>& rhs)
{
    return lhs -= rhs;
}

template <typename T,
          size_t N,
          typename Scalar,
          typename = typename std::enable_if<std::is_arithmetic<Scalar>::value,
                                             Scalar>::type>
Point<T, N> operator+(Point<T, N> lhs, Scalar s)
{
    return lhs += s;
}

template <typename T,
          size_t N,
          typename Scalar,
          typename = typename std::enable_if<std::is_arithmetic<Scalar>::value,
                                             Scalar>::type>
Point<T, N> operator+(Scalar s, Point<T, N> lhs)
{
    return lhs + s;
}

template <typename T,
          size_t N,
          typename Scalar,
          typename = typename std::enable_if<std::is_arithmetic<Scalar>::value,
                                             Scalar>::type>
Point<T, N> operator-(Point<T, N> lhs, Scalar s)
{
    return lhs -= s;
}

template <typename T,
          size_t N,
          typename Scalar,
          typename = typename std::enable_if<std::is_arithmetic<Scalar>::value,
                                             Scalar>::type>
Point<T, N> operator-(Scalar s, Point<T, N> lhs)
{
    return Point<T, N>::fill(T{s}) - lhs;
}

template <typename T,
          size_t N,
          typename Scalar,
          typename = typename std::enable_if<std::is_arithmetic<Scalar>::value,
                                             Scalar>::type>
Point<T, N> operator*(Point<T, N> lhs, Scalar s)
{
    return lhs *= s;
}

template <typename T,
          size_t N,
          typename Scalar,
          typename = typename std::enable_if<std::is_arithmetic<Scalar>::value,
                                             Scalar>::type>
Point<T, N> operator*(Scalar s, Point<T, N> lhs)
{
    return lhs * s;
}

template <typename T,
          size_t N,
          typename Scalar,
          typename = typename std::enable_if<std::is_arithmetic<Scalar>::value,
                                             Scalar>::type>
Point<T, N> operator/(Point<T, N> lhs, Scalar s)
{
    return lhs /= s;
}

// Comparison operations
template <typename T, size_t N>
bool operator==(const Point<T, N>& lhs, const Point<T, N>& rhs)
{
    auto zipped = zip(lhs, rhs);
    return std::all_of(std::begin(zipped), std::end(zipped),
                       [](boost::tuple<T, T> t) {
                           return boost::get<0>(t) == boost::get<1>(t);
                       });
}

template <typename T, size_t N>
bool operator!=(const Point<T, N>& lhs, const Point<T, N>& rhs)
{
    return !operator==(lhs, rhs);
}

template <typename T, size_t N>
bool operator<(const Point<T, N>& lhs, const Point<T, N>& rhs)
{
    return lhs.norm() < rhs.norm();
}

template <typename T, size_t N>
bool operator>(const Point<T, N>& lhs, const Point<T, N>& rhs)
{
    return rhs < lhs;
}

template <typename T, size_t N>
bool operator<=(const Point<T, N>& lhs, const Point<T, N>& rhs)
{
    return !operator>(lhs, rhs);
}

template <typename T, size_t N>
bool operator>=(const Point<T, N>& lhs, const Point<T, N>& rhs)
{
    return !operator<(lhs, rhs);
}

// I/O ops
template <typename T, size_t N>
std::istream& operator>>(std::istream& s, Point<T, N>& p)
{
    T tmp;
    for (size_t i = 0; i < N; ++i) {
        s >> tmp;
        p[i] = tmp;
    }
    return s;
}

template <typename T, size_t N>
std::ostream& operator<<(std::ostream& s, const Point<T, N>& p)
{
    for (auto it = std::begin(p); it != std::end(p) - 1; ++it) {
        s << *it << " ";
    }
    return s << p.back();
}
}
