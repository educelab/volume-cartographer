#pragma once

#include <cassert>
#include <iostream>
#include <stdexcept>
#include "common/types/Exceptions.h"
#include "common/types/PointSet.h"

namespace volcart
{
template <typename T>
class OrderedPointSet : public PointSet<T>
{
public:
    using BaseClass = PointSet<T>;
    using BaseClass::BaseClass;
    using BaseClass::_data;

    // Could use a better way to determine the multiplier
    constexpr static size_t CAPACITY_MULTIPLIER = 20;

    explicit OrderedPointSet() : BaseClass(), _width(0) {}
    explicit OrderedPointSet(size_t width) : BaseClass(), _width(width)
    {
        _data.reserve(_width * CAPACITY_MULTIPLIER);
    }
    explicit OrderedPointSet(size_t width, T initVal)
        : BaseClass(), _width(width)
    {
        _data.assign(_width * CAPACITY_MULTIPLIER, initVal);
    }

    // Fill static method
    static OrderedPointSet Fill(size_t width, size_t height, T initVal)
    {
        OrderedPointSet ps(width);
        std::vector<T> v;
        v.assign(width, initVal);
        for (size_t _ = 0; _ < height; ++_) {
            ps.pushRow(v);
        }
        return ps;
    }

    // 2D access
    // NOTE: x, then y
    const T& operator()(size_t x, size_t y) const
    {
        assert(x < _width && "x out of range");
        assert(y * _width + x < _data.size() && "(x, y) out of range");
        return _data[y * _width + x];
    }
    T& operator()(size_t x, size_t y)
    {
        assert(x < _width && "x out of range");
        assert(y * _width + x < _data.size() && "(x, y) out of range");
        return _data[y * _width + x];
    }
    size_t width() const { return _width; }

    // _data.size() should be a perfect multiple of _width, so this should
    // return a whole integer
    size_t height() const { return (_width == 0 ? 0 : this->size() / _width); }

    // Push a row of points to the OrderedPointSet
    void pushRow(const std::vector<T>& points)
    {
        assert(points.size() == _width && "row incorrect size");
        std::copy(
            std::begin(points), std::end(points), std::back_inserter(_data));
    }
    void pushRow(std::vector<T>&& points)
    {
        assert(points.size() == _width && "row incorrect size");
        std::copy(
            std::begin(points), std::end(points), std::back_inserter(_data));
    }

    // Not implemented for this class
    void push_back(const T& val) = delete;
    void push_back(T&& val) = delete;

    // Append one pointset to another
    // NOTE: Overrides PointSet<T>::append
    void append(const OrderedPointSet<T>& ps)
    {
        // ps must be same width as this pointset
        if (_width != ps.width()) {
            auto msg = "Cannot append pointset with different width";
            throw std::logic_error(msg);
        }

        std::copy(std::begin(ps), std::end(ps), std::back_inserter(_data));
    }

    // Get a particular row
    std::vector<T> getRow(size_t i) const
    {
        if (i >= this->height()) {
            throw std::range_error("out of range");
        }
        std::vector<T> row(_width);
        std::copy(
            std::begin(_data) + _width * i,
            std::begin(_data) + _width * (i + 1), std::begin(row));
        return row;
    }

    // Copy rows to a new pointset. Copies rows [i, j]
    OrderedPointSet copyRows(size_t i, size_t j) const
    {
        if (i >= this->height() || j >= this->height()) {
            throw std::range_error("out of range");
        } else if (i > j) {
            throw std::logic_error("i must be less than j");
        }
        OrderedPointSet ps(_width);
        std::copy(
            std::begin(_data) + _width * i,
            std::begin(_data) + _width * (j + 1),
            std::back_inserter(ps.data()));
        return ps;
    }

private:
    size_t _width;
};
}
