#pragma once

#include <algorithm>
#include <cassert>
#include <iostream>
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

    PointSet() : _width(0), _height(0), _capacity(0), _ordered(false), _data()
    {
    }

    explicit PointSet(size_t size)
        : _width(0), _height(0), _capacity(size), _ordered(false)
    {
        _data.reserve(_capacity);
    }

    explicit PointSet(size_t width, size_t height)
        : _width(width)
        , _height(height)
        , _capacity(width * height)
        , _ordered(true)
    {
        _data.reserve(_capacity);
    }

    explicit PointSet(size_t width, size_t height, T init_val)
        : _width(width)
        , _height(height)
        , _capacity(width * height)
        , _ordered(true)
    {
        _data.assign(_capacity, init_val);
    }

    // Linear access - no concept of 2D layout
    const T& operator[](size_t idx) const { return _data[idx]; }
    T& operator[](size_t idx) { return _data[idx]; }

    // 2D access
    // NOTE: x, then y
    const T& operator()(size_t x, size_t y) const
    {
        assert(_ordered && "Cannot use operator() with unordered PointSet");
        return _data[y * _width + x];
    }
    T& operator()(size_t x, size_t y)
    {
        assert(_ordered && "Cannot use operator() with unordered PointSet");
        return _data[y * _width + x];
    }

    // Metadata
    size_t width() const { return _width; }
    size_t height() const { return _height; }
    size_t size() const { return _data.size(); }
    size_t capacity() const { return _capacity; }
    bool empty() const { return _data.empty(); }
    bool isOrdered() const { return _ordered; }
    void setOrdered(size_t newWidth, size_t newHeight)
    {
        _ordered = true;
        _width = newWidth;
        _height = newHeight;
        _capacity = newWidth * newHeight;
    }
    void setUnordered()
    {
        _ordered = false;
        _width = _height = 0;
    }

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
        assert(_data.size() < _capacity && "PointSet full");
        _data.push_back(val);
    }
    void push_back(T&& val)
    {
        assert(_data.size() < _capacity && "PointSet full");
        _data.push_back(val);
    }
    void push_row(const std::vector<T>& points)
    {
        assert(points.size() == _width && "row incorrect size");
        assert(_data.size() < _capacity && "PointSet full");
        std::copy(std::begin(points), std::end(points),
                  std::back_inserter(_data));
    }
    void push_row(std::vector<T>&& points)
    {
        assert(points.size() == _width && "row incorrect size");
        assert(_data.size() < _capacity && "PointSet full");
        std::copy(std::begin(points), std::end(points),
                  std::back_inserter(_data));
    }

private:
    size_t _width;
    size_t _height;
    size_t _capacity;
    bool _ordered;
    Container _data;
};
}
