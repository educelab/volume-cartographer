#pragma once

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace volcart
{
template <typename T>
class PointSet
{
public:
    using Point = T;
    using Container = std::vector<Point>;
    using Iterator = typename Container::iterator;
    using ConstIterator = typename Container::const_iterator;
    constexpr static int FORMAT_VERSION = 1;
    constexpr static auto HEADER_TERMINATOR = "<>";
    constexpr static auto HEADER_TERMINATOR_REGEX = "^<>$";

    explicit PointSet() : data_{} {}
    explicit PointSet(size_t initSize) { data_.reserve(initSize); }
    explicit PointSet(size_t initSize, T initVal)
    {
        data_.assign(initSize, initVal);
    }

    // Fill static method
    static PointSet Fill(size_t initSize, T initVal)
    {
        return PointSet(initSize, initVal);
    }

    // Linear access - no concept of 2D layout
    const T& operator[](size_t idx) const
    {
        assert(idx < data_.size() && "idx out of range");
        return data_[idx];
    }
    T& operator[](size_t idx)
    {
        assert(idx < data_.size() && "idx out of range");
        return data_[idx];
    }

    // Metadata
    size_t size() const { return data_.size(); }
    bool empty() const { return data_.empty(); }
    std::vector<T>& data() { return data_; }

    // Iterators and element accessors
    Iterator begin() { return std::begin(data_); }
    ConstIterator begin() const { return std::begin(data_); }
    Iterator end() { return std::end(data_); }
    ConstIterator end() const { return std::end(data_); }
    T& front() { return data_.front(); }
    const T& front() const { return data_.front(); }
    T& back() { return data_.back(); }
    const T& back() const { return data_.back(); }

    // Some basic statistics
    T min() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::min_element(
            std::begin(data_), std::end(data_),
            [](auto lhs, auto rhs) { return cv::norm(lhs) < cv::norm(rhs); });
    }
    T max() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::max_element(
            std::begin(data_), std::end(data_),
            [](auto lhs, auto rhs) { return cv::norm(lhs) < cv::norm(rhs); });
    }
    std::pair<T, T> minMax() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        auto pair = std::minmax_element(
            std::begin(data_), std::end(data_),
            [](auto lhs, auto rhs) { return cv::norm(lhs) < cv::norm(rhs); });
        return {*pair.first, *pair.second};
    }

    // Add elements
    void push_back(const T& val) { data_.push_back(val); }
    void push_back(T&& val) { data_.push_back(val); }
    void append(const PointSet<T>& ps)
    {
        std::copy(std::begin(ps), std::end(ps), std::back_inserter(data_));
    }

    void clear() { data_.clear(); }

protected:
    Container data_;
};
}  // namespace volcart
