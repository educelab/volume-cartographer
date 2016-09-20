#pragma once

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

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

    explicit PointSet() : _data(), _capacity(0) {}
    explicit PointSet(size_t size) : _capacity(size) { _data.reserve(size); }
    explicit PointSet(size_t size, T initVal) : _capacity(size)
    {
        _data.assign(size, initVal);
    }

    // Fill static method
    static PointSet Fill(size_t size, T initVal)
    {
        return PointSet(size, initVal);
    }

    // Linear access - no concept of 2D layout
    const T& operator[](size_t idx) const { return _data[idx]; }
    T& operator[](size_t idx) { return _data[idx]; }

    // Metadata
    size_t size() const { return _data.size(); }
    size_t capacity() const { return _capacity; }
    bool empty() const { return _data.empty(); }

    // Iterators and element accessors
    Iterator begin() { return std::begin(_data); }
    ConstIterator begin() const { return std::begin(_data); }
    Iterator end() { return std::end(_data); }
    ConstIterator end() const { return std::end(_data); }
    T& front() { return _data.front(); }
    const T& front() const { return _data.front(); }
    T& back() { return _data.back(); }
    const T& back() const { return _data.back(); }

    // Some basic statistics
    T min() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::min_element(std::begin(_data), std::end(_data));
    }
    T max() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::max_element(std::begin(_data), std::end(_data));
    }
    std::pair<T, T> min_max() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        auto pair = std::minmax_element(std::begin(_data), std::end(_data));
        return {*pair.first, *pair.second};
    }

    // Add elements
    void push_back(const T& val)
    {
        if (_data.size() >= _capacity) {
            auto msg = "PointSet is full";
            throw std::logic_error(msg);
        }
        _data.push_back(val);
    }
    void push_back(T&& val)
    {
        if (_data.size() >= _capacity) {
            auto msg = "PointSet is full";
            throw std::logic_error(msg);
        }
        _data.push_back(val);
    }
    void append(const PointSet<T>& ps)
    {
        _capacity = _capacity + ps.capacity();
        std::copy(std::begin(ps), std::end(ps), std::back_inserter(_data));
    }

protected:
    Container _data;
    size_t _capacity;
};
}
