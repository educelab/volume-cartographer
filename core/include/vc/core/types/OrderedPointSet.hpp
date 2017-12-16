#pragma once

#include <cassert>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "Exceptions.hpp"
#include "PointSet.hpp"

namespace volcart
{
/**
 * @class OrderedPointSet
 * @brief Holds a collection of ordered points
 * @author Sean Karlage
 *
 * An OrderedPointSet stores points with 2D array-like adjacency information
 * that is independent of the Euclidean position of the points themselves. Point
 * ordering provides extra topological information that may not be obvious from
 * positional information alone.
 *
 * It is currently used by the propagation-based segmentation algorithms
 * (LRPS, STPS), where an entire row of points propagates through the volume for
 * a discrete number of iterations. The path a single point takes through the
 * volume is given by a single column of the resulting OrderedPointSet.
 *
 * An OrderedPointSet can only grow in the number of rows that it has. To
 * change the width of the point set, make a new OrderedPointSet with the
 * desired width or use reset() in conjunction with setWidth().
 *
 * In order to use the PointSetIO functions, the point type should be based on
 * `int`, `float`, or `double` (e.g. `cv::Vec2i`, `cv::Vec3d`, etc.)
 *
 * @ingroup Types
 */
template <typename T>
class OrderedPointSet : public PointSet<T>
{
public:
    using BaseClass = PointSet<T>;
    using BaseClass::BaseClass;
    using BaseClass::data_;

    /** Pointer type */
    using Pointer = std::shared_ptr<OrderedPointSet<T>>;

    /**@{*/
    /** @brief Default constructor */
    explicit OrderedPointSet() = default;

    /** @brief Constructor with width parameter */
    explicit OrderedPointSet(size_t width) : BaseClass(), width_(width)
    {
        data_.reserve(width_ * CAPACITY_MULTIPLIER);
    }

    /** @brief Constructor with width parameter and initialization value */
    explicit OrderedPointSet(size_t width, T initVal)
        : BaseClass(), width_(width)
    {
        data_.assign(width_ * CAPACITY_MULTIPLIER, initVal);
    }
    /**@}*/

    /**@{*/
    /** @brief Get a point from the OrderedPointSet at row y, column x */
    const T& operator()(size_t y, size_t x) const
    {
        assert(x < width_ && "x out of range");
        assert(y * width_ + x < data_.size() && "(x, y) out of range");
        return data_[y * width_ + x];
    }

    /** @copybrief OrderedPointSet::operator()() */
    T& operator()(size_t y, size_t x)
    {
        assert(x < width_ && "x out of range");
        assert(y * width_ + x < data_.size() && "(x, y) out of range");
        return data_[y * width_ + x];
    }
    /**@}*/

    /**@{*/
    /** @brief Return the number of columns in the OrderedPointSet */
    size_t width() const { return width_; }

    /** @brief Return the number of rows in the OrderedPointSet */
    // data_.size() should be a perfect multiple of width_, so this should
    // return a whole integer
    size_t height() const { return (width_ == 0 ? 0 : this->size() / width_); }

    /** @brief Set the ordering width
     *
     * This class provides no ability to resize the OrderedPointSet's width
     * after it has been set. To reuse the same OrderedPointSet object with a
     * different width value, use reset() before calling this function.
     *
     * This function throws a std::logic_error if the width has already been
     * set.
     */
    void setWidth(size_t width)
    {
        if (width_ != 0) {
            auto msg = "Cannot change width if already set";
            throw std::logic_error(msg);
        }
        width_ = width;
    }

    /**
     * @brief Like clear(), but zeroes the OrderedPointSet width so that it can
     * be redefined
     */
    void reset()
    {
        width_ = 0;
        this->clear();
    }
    /**@}*/

    /**@{*/
    /** @brief Add a row of points to the OrderedPointSet */
    void pushRow(const std::vector<T>& points)
    {
        assert(points.size() == width_ && "row incorrect size");
        std::copy(
            std::begin(points), std::end(points), std::back_inserter(data_));
    }

    /** @copydoc OrderedPointSet::pushRow() */
    void pushRow(std::vector<T>&& points)
    {
        assert(points.size() == width_ && "row incorrect size");
        std::copy(
            std::begin(points), std::end(points), std::back_inserter(data_));
    }

    // Cannot add individual points to this class because it would break
    // width constraint calculation
    void push_back(const T& val) = delete;
    void push_back(T&& val) = delete;

    /**
     * @brief Append an OrderedPointSet to the end of the current one
     *
     * Throws a std::logic_error if the widths of the two point sets do not
     * match.
     */
    void append(const OrderedPointSet<T>& ps)
    {
        // ps must be same width as this pointset
        if (width_ != ps.width()) {
            auto msg = "Cannot append pointset with different width";
            throw std::logic_error(msg);
        }

        std::copy(std::begin(ps), std::end(ps), std::back_inserter(data_));
    }

    /** @brief Get a row of points
     *
     * Throws a std::range_error if `i` is outside the range of row indices.
     */
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

    /**
     * @brief Get multiple rows of points
     *
     * Copies rows [i, j).
     *
     * Throws a std::range_error if `i` is outside the range of row
     * indices. Throws std::logic_error if `j <= i`.
     */
    OrderedPointSet copyRows(size_t i, size_t j) const
    {
        if (i >= this->height() || j > this->height()) {
            throw std::range_error("out of range");
        } else if (j <= i) {
            throw std::logic_error("i must be less than j");
        }
        OrderedPointSet ps(width_);
        std::copy(
            std::begin(data_) + width_ * i, std::begin(data_) + width_ * j,
            std::back_inserter(ps.data()));
        return ps;
    }
    /**@}*/

    /**@{*/
    /** @brief Create an OrderedPointSet of a specific size, filled with an
     * initial value */
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
    /**@{*/

private:
    /** Number of columns */
    size_t width_{0};
    /** Number of rows preallocated */
    constexpr static size_t CAPACITY_MULTIPLIER = 20;
};
}  // namespace volcart
