#pragma once

#include <type_traits>

#include <opencv2/core.hpp>

namespace volcart
{

template <
    typename T,
    int dim,
    typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
class BoundingBox
{
public:
    using Point = cv::Vec<T, dim>;

    BoundingBox() = default;

    BoundingBox(Point lower, Point upper) : p0_{lower}, p1_{upper} {}

    Point getLowerBound() const { return p0_; }
    Point getUpperBound() const { return p1_; }

    T getLowerBoundByIndex(int i)
    {
        if (i >= dim) {
            throw std::domain_error("Invalid dimension index");
        }
        return p0_[i];
    }

    T getUpperBoundByIndex(int i)
    {
        if (i >= dim) {
            throw std::domain_error("Invalid dimension index");
        }
        return p1_[i];
    }

    void setLowerBound(Point p) { p0_ = std::move(p); }
    void setUpperBound(Point p) { p1_ = std::move(p); }

    void setLowerBoundByIndex(int i, T val)
    {
        if (i >= dim) {
            throw std::domain_error("Invalid dimension index");
        }
        p0_[i] = val;
    }

    void setUpperBoundByIndex(int i, T val)
    {
        if (i >= dim) {
            throw std::domain_error("Invalid dimension index");
        }
        p1_[i] = val;
    }

    bool isInBounds(const Point p) const
    {
        for (int d = 0; d < dim; d++) {
            if (p[d] < p0_[d] || p[d] >= p1_[d]) {
                return false;
            }
        }
        return true;
    }

private:
    Point p0_{0, 0, 0};
    Point p1_{0, 0, 0};
};
}