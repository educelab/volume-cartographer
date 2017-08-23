#pragma once

#include <type_traits>

#include <opencv2/core.hpp>

namespace volcart
{

/**
 * @class BoundingBox
 * @author Seth Parker
 * @brief Generic axis-aligned bounding box class for operations in N-dimensions
 *
 * Provides functions for inside/outside box checks. Lower boundary is inclusive
 * and upper boundary is exclusive: `[lower, upper)`
 *
 * @tparam T Fundamental type of coordinate system
 * @tparam Dim Number of dimensions
 */
template <
    typename T,
    int Dim,
    typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
class BoundingBox
{
public:
    /** Bounding box position type */
    using Point = cv::Vec<T, Dim>;

    /** @brief Default constructor */
    BoundingBox() = default;

    /** @brief Constructor with lower and upper bounds initialization */
    BoundingBox(Point lower, Point upper) : p0_{lower}, p1_{upper} {}

    /** @brief Get the lower boundary for each axis */
    Point getLowerBound() const { return p0_; }

    /** @brief Get the upper boundary for each axis */
    Point getUpperBound() const { return p1_; }

    /** @brief Get the lower boundary for a specific axis by axis index */
    T getLowerBoundByIndex(int i)
    {
        if (i >= Dim) {
            throw std::domain_error("Invalid dimension index");
        }
        return p0_[i];
    }

    /** @brief Get the upper boundary for a specific axis by axis index */
    T getUpperBoundByIndex(int i)
    {
        if (i >= Dim) {
            throw std::domain_error("Invalid dimension index");
        }
        return p1_[i];
    }

    /** @brief Set the lower boundary for each axis */
    void setLowerBound(Point p) { p0_ = std::move(p); }

    /** @brief Set the upper boundary for each axis */
    void setUpperBound(Point p) { p1_ = std::move(p); }

    /** @brief Set the lower boundary for a specific axis by axis index */
    void setLowerBoundByIndex(int i, T val)
    {
        if (i >= Dim) {
            throw std::domain_error("Invalid dimension index");
        }
        p0_[i] = val;
    }

    /** @brief Set the upper boundary for a specific axis by axis index */
    void setUpperBoundByIndex(int i, T val)
    {
        if (i >= Dim) {
            throw std::domain_error("Invalid dimension index");
        }
        p1_[i] = val;
    }

    /** @brief Check if a Point is within the bounds of the box */
    bool isInBounds(const Point& p) const
    {
        for (int d = 0; d < Dim; d++) {
            if (p[d] < p0_[d] || p[d] >= p1_[d]) {
                return false;
            }
        }
        return true;
    }

private:
    /** Lower bound */
    Point p0_{0, 0, 0};
    /** Upper bound */
    Point p1_{0, 0, 0};
};
}