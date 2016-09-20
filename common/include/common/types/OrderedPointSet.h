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
    using BaseClass::_capacity;

    explicit OrderedPointSet() : BaseClass(), _width(0), _height(0) {}
    explicit OrderedPointSet(size_t width, size_t height)
        : BaseClass(), _width(width), _height(height)
    {
        _capacity = width * height;
        _data.reserve(_capacity);
    }
    explicit OrderedPointSet(size_t width, size_t height, T initVal)
        : BaseClass(), _width(width), _height(height)
    {
        _capacity = width * height;
        _data.assign(_capacity, initVal);
    }

    // Fill static method
    static OrderedPointSet Fill(size_t width, size_t height, T initVal)
    {
        return OrderedPointSet(width, height, initVal);
    }

    // 2D access
    // NOTE: x, then y
    const T& operator()(size_t x, size_t y) const
    {
        return _data[y * _width + x];
    }
    T& operator()(size_t x, size_t y) { return _data[y * _width + x]; }
    size_t width() const { return _width; }
    size_t height() const { return _height; }

    // Push a row of points to the OrderedPointSet
    void push_row(const std::vector<T>& points)
    {
        assert(points.size() == _width && "row incorrect size");
        assert(_data.size() < _capacity && "PointSet full");
        std::copy(
            std::begin(points), std::end(points), std::back_inserter(_data));
    }
    void push_row(std::vector<T>&& points)
    {
        assert(points.size() == _width && "row incorrect size");
        assert(_data.size() < _capacity && "PointSet full");
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

        _height = (this->size() / _width) + ps.height();
        std::copy(std::begin(ps), std::end(ps), std::back_inserter(_data));
        _capacity += ps.capacity();
    }

private:
    size_t _width;
    size_t _height;
};
}
