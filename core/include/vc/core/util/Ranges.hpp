#pragma once

#include <cmath>
#include <iostream>
#include <type_traits>

namespace volcart
{

/**
 * @brief Provides an iterable range of numbers with optional step size
 *
 * Convenience class for generating range-based for loops over numerical values.
 * Note that this class only returns values and const references.
 *
 * @tparam T Integral or floating-point type
 *
 * @ingroup Util
 */
template <typename T, std::enable_if_t<std::is_arithmetic<T>::value, int> = 0>
class RangeIterable
{
private:
    template <typename ItT>
    class CountingIterator
    {
    private:
        /** Current iterator value */
        ItT value_{0};
        /** Step size */
        ItT step_{1};

    public:
        /** @{ Iterator type traits */
        using difference_type = size_t;
        using value_type = ItT;
        using pointer = value_type*;
        using reference = value_type const&;
        using iterator_category = std::input_iterator_tag;
        /** @} */

        /** Construct with starting value and step size */
        explicit CountingIterator(ItT value, ItT step)
            : value_{value}, step_{step}
        {
        }

        /** Get the current value */
        reference operator*() const { return value_; }

        /** Equality comparison */
        bool operator==(const CountingIterator& other) const
        {
            // This assumes we only ever compare against end()
            return value_ >= other.value_;
        }

        /** Inequality comparison */
        bool operator!=(const CountingIterator& other) const
        {
            return !(*this == other);
        }

        /** Increment operator */
        CountingIterator& operator++()
        {
            value_ += step_;
            return *this;
        }
    };

public:
    /** Iterator type */
    using iterator = CountingIterator<T>;
    /** Const iterator type */
    using const_iterator = CountingIterator<T const&>;

    /** Construct a new range */
    RangeIterable(T start, T end, T step)
        : start_{start}, end_{end}, step_{step}
    {
        if (step <= 0) {
            throw std::invalid_argument("step size cannot be zero or negative");
        }
    }

    /** Return the start of the range */
    iterator begin() const { return iterator{start_, step_}; }
    /** Return the end of the range */
    iterator end() const { return iterator{end_, step_}; }
    /** Return the const start of the range */
    const_iterator cbegin() const { return const_iterator{start_, step_}; }
    /** Return the const end of the range */
    const_iterator cend() const { return const_iterator{end_, step_}; }

    /** Returns the size of the range (floating point ranges) */
    template <typename Q = T>
    std::enable_if_t<std::is_floating_point<Q>::value, size_t> size() const
    {
        return static_cast<size_t>(std::ceil((end_ - start_) / step_));
    }

    /** Returns the size of the range (integral ranges) */
    template <typename Q = T>
    std::enable_if_t<std::is_integral<Q>::value, size_t> size() const
    {
        return static_cast<size_t>((end_ - start_) / step_);
    }

private:
    T start_{0};
    T end_{0};
    T step_{1};
};

/** Convenience method for printing RangeIterables */
template <typename T>
std::ostream& operator<<(std::ostream& os, const RangeIterable<T>& it)
{
    for (const auto& v : it) {
        if (v != *it.begin()) {
            os << " ";
        }
        os << v;
    }
    return os;
}

/**
 * Convenience method for constructing iterable ranges. Starting value is 0.
 *
 * @param stop ending value (non-inclusive)
 * @return RangeIterable<T>
 *
 * @ingroup Util
 */
template <typename T>
auto Range(T stop)
{
    return RangeIterable<T>(0, stop, 1);
}

/**
 * Convenience method for constructing iterable ranges. Starting value is 0.
 *
 * @param stop ending value (non-inclusive)
 * @return RangeIterable<T>
 *
 * @ingroup Util
 */
template <typename T0, typename T1>
auto Range(T0 start, T1 stop)
{
    return RangeIterable<T0>(start, stop, 1);
}

/**
 * Convenience method for constructing iterable ranges.
 *
 * @param start starting value
 * @param stop ending value (non-inclusive)
 * @param step step size
 * @return RangeIterable<T0>
 *
 * @ingroup Util
 */
template <typename T0, typename T1, typename T2>
auto Range(T0 start, T1 stop, T2 step)
{
    return RangeIterable<T0>(start, stop, step);
}

}  // namespace volcart