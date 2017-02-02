#pragma once

#include <cassert>
#include <iostream>
#include <stdexcept>

#include "core/types/Exceptions.hpp"
#include "core/types/PointSet.hpp"

namespace volcart
{

template <typename T>
class OrderedPointSet : public PointSet<T>
{
public:
    using BaseClass = PointSet<T>;
    using BaseClass::BaseClass;
    using BaseClass::data_;

    // Could use a better way to determine the multiplier
    constexpr static size_t CAPACITY_MULTIPLIER = 20;

    explicit OrderedPointSet() : BaseClass(), width_(0) {}
    explicit OrderedPointSet(size_t width) : BaseClass(), width_(width)
    {
        data_.reserve(width_ * CAPACITY_MULTIPLIER);
    }
    explicit OrderedPointSet(size_t width, T initVal)
        : BaseClass(), width_(width)
    {
        data_.assign(width_ * CAPACITY_MULTIPLIER, initVal);
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
    const T& operator()(size_t y, size_t x) const
    {
        assert(x < width_ && "x out of range");
        assert(y * width_ + x < data_.size() && "(x, y) out of range");
        return data_[y * width_ + x];
    }
    T& operator()(size_t y, size_t x)
    {
        assert(x < width_ && "x out of range");
        assert(y * width_ + x < data_.size() && "(x, y) out of range");
        return data_[y * width_ + x];
    }
    size_t width() const { return width_; }

    // data_.size() should be a perfect multiple of width_, so this should
    // return a whole integer
    size_t height() const { return (width_ == 0 ? 0 : this->size() / width_); }

    // Resize the width
    void setWidth(size_t width)
    {
        if (width_ != 0) {
            auto msg = "Cannot change width if already set";
            throw std::logic_error(msg);
        }
        width_ = width;
    }

    void reset()
    {
        width_ = 0;
        this->clear();
    }

    // Push a row of points to the OrderedPointSet
    void pushRow(const std::vector<T>& points)
    {
        assert(points.size() == width_ && "row incorrect size");
        std::copy(
            std::begin(points), std::end(points), std::back_inserter(data_));
    }
    void pushRow(std::vector<T>&& points)
    {
        assert(points.size() == width_ && "row incorrect size");
        std::copy(
            std::begin(points), std::end(points), std::back_inserter(data_));
    }

    // Not implemented for this class
    void push_back(const T& val) = delete;
    void push_back(T&& val) = delete;

    // Append one pointset to another
    // NOTE: Overrides PointSet<T>::append
    void append(const OrderedPointSet<T>& ps)
    {
        // ps must be same width as this pointset
        if (width_ != ps.width()) {
            auto msg = "Cannot append pointset with different width";
            throw std::logic_error(msg);
        }

        std::copy(std::begin(ps), std::end(ps), std::back_inserter(data_));
    }

    // Get a particular row
    std::vector<T> getRow(size_t i) const
    {
        if (i >= this->height()) {
            throw std::range_error("out of range");
        }
        std::vector<T> row(width_);
        std::copy(
            std::begin(data_) + width_ * i,
            std::begin(data_) + width_ * (i + 1), std::begin(row));
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
        OrderedPointSet ps(width_);
        std::copy(
            std::begin(data_) + width_ * i,
            std::begin(data_) + width_ * (j + 1),
            std::back_inserter(ps.data()));
        return ps;
    }

private:
    size_t width_;
};
}  // namespace volcart
