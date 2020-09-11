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
        /** Max iterator value */
        ItT maxValue_{1};
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
        explicit CountingIterator(ItT value, ItT maxValue, ItT step)
            : value_{value}, maxValue_{maxValue}, step_{step}
        {
        }

        /** Get the current value */
        reference operator*() const { return value_; }

        /** Equality comparison */
        bool operator==(const CountingIterator& other) const
        {
            return value_ == other.value_;
        }

        /** Inequality comparison */
        bool operator!=(const CountingIterator& other) const
        {
            return !(*this == other);
        }

        /** Increment operator */
        CountingIterator& operator++()
        {
            // Stop iterating at the max value
            if (value_ < maxValue_) {
                value_ = std::min(value_ + step_, maxValue_);
            }
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
    iterator begin() const { return iterator{start_, end_, step_}; }
    /** Return the end of the range */
    iterator end() const { return iterator{end_, end_, step_}; }
    /** Return the const start of the range */
    const_iterator cbegin() const
    {
        return const_iterator{start_, end_, step_};
    }
    /** Return the const end of the range */
    const_iterator cend() const { return const_iterator{end_, end_, step_}; }

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
auto range(T stop)
{
    return RangeIterable<T>(0, stop, 1);
}

/**
 * Convenience method for constructing iterable ranges. Step value is 1.
 *
 * @param stop ending value (non-inclusive)
 * @return RangeIterable<T>
 *
 * @ingroup Util
 */
template <typename T0, typename T1>
auto range(T0 start, T1 stop)
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
auto range(T0 start, T1 stop, T2 step)
{
    return RangeIterable<T0>(start, stop, step);
}

/**
 * @brief Provides an iterable range of 2D number pairs with optional step size
 *
 * Convenience class for generating range-based for loops over 2D values.
 * Note that this class only returns values and const references.
 *
 * @tparam T Integral or floating-point type
 *
 * @ingroup Util
 */
template <typename T, std::enable_if_t<std::is_arithmetic<T>::value, int> = 0>
class Range2DIterable
{
private:
    template <typename ItT>
    class Counting2DIterator
    {
    private:
        /** Current iterator V value */
        ItT v_{0};
        /** V loop limit */
        ItT vMax_{1};
        /** Current iterator U value */
        ItT u_{0};
        /** Starting value */
        ItT uStart_{0};
        /** U loop limit */
        ItT uMax_{1};
        /** Step size */
        ItT step_{1};

    public:
        /** @{ Iterator type traits */
        using difference_type = size_t;
        using value_type = std::pair<ItT const&, ItT const&>;
        using pointer = value_type*;
        using reference = value_type;
        using iterator_category = std::input_iterator_tag;
        /** @} */

        /** Construct with starting values and step size */
        explicit Counting2DIterator(ItT v, ItT vMax, ItT u, ItT uMax, ItT step)
            : v_{v}, vMax_{vMax}, u_{u}, uStart_{u}, uMax_{uMax}, step_{step}
        {
        }

        /** Get the current value */
        reference operator*() const { return {v_, u_}; }

        /** Equality comparison */
        bool operator==(const Counting2DIterator& other) const
        {
            return u_ == other.u_ and v_ == other.v_;
        }

        /** Inequality comparison */
        bool operator!=(const Counting2DIterator& other) const
        {
            return !(*this == other);
        }

        /** Increment operator */
        Counting2DIterator& operator++()
        {
            if (u_ < uMax_) {
                u_ = std::min(u_ + step_, uMax_);
            }

            if (u_ == uMax_ and v_ < vMax_) {
                v_ = std::min(v_ + step_, vMax_);

                if (v_ < vMax_) {
                    u_ = uStart_;
                }
            }

            return *this;
        }
    };

public:
    /** Iterator type */
    using iterator = Counting2DIterator<T>;
    /** Const iterator type */
    using const_iterator = Counting2DIterator<T const&>;

    /** Construct a new range */
    Range2DIterable(T vStart, T vEnd, T uStart, T uEnd, T step)
        : vStart_{vStart}
        , vEnd_{vEnd}
        , uStart_{uStart}
        , uEnd_{uEnd}
        , step_{step}
    {
        if (step <= 0) {
            throw std::invalid_argument("step size cannot be zero or negative");
        }
    }

    /** Return the start of the range */
    iterator begin() const
    {
        return iterator{vStart_, vEnd_, uStart_, uEnd_, step_};
    }
    /** Return the end of the range */
    iterator end() const { return iterator{vEnd_, vEnd_, uEnd_, uEnd_, step_}; }
    /** Return the const start of the range */
    const_iterator cbegin() const
    {
        return const_iterator{vStart_, vEnd_, uStart_, uEnd_, step_};
    }
    /** Return the const end of the range */
    const_iterator cend() const
    {
        return const_iterator{vEnd_, vEnd_, uEnd_, uEnd_, step_};
    }

    /** Returns the size of the range (floating point ranges) */
    template <typename Q = T>
    std::enable_if_t<std::is_floating_point<Q>::value, size_t> size() const
    {
        auto vLen = static_cast<size_t>(std::ceil((vEnd_ - vStart_) / step_));
        return static_cast<size_t>(std::ceil((uEnd_ - uStart_) / step_)) * vLen;
    }

    /** Returns the size of the range (integral ranges) */
    template <typename Q = T>
    std::enable_if_t<std::is_integral<Q>::value, size_t> size() const
    {
        auto vLen = static_cast<size_t>((vEnd_ - vStart_) / step_);
        return static_cast<size_t>((uEnd_ - uStart_) / step_) * vLen;
    }

private:
    /** Outer loop starting value */
    T vStart_{0};
    /** Outer loop limit */
    T vEnd_{0};
    /** Inner loop starting value */
    T uStart_{0};
    /** Inner loop limit */
    T uEnd_{0};
    /** Loop step size */
    T step_{1};
};

/**
 * Convenience method for constructing iterable 2D ranges. Starting value is 0.
 *
 * @param vStop Upper limit for outer loop
 * @param uStop Upper limit for inner loop
 * @return Range2DIterable<T0>
 *
 * @ingroup Util
 */
template <typename T0, typename T1>
auto range2D(T0 vStop, T1 uStop)
{
    return Range2DIterable<T0>(0, vStop, 0, uStop, 1);
}

/**
 * Convenience method for constructing iterable 2D ranges. Step value is 1.
 *
 * @param vStart Starting value for outer loop index
 * @param vStop Upper limit for outer loop
 * @param uStart Starting value for inner loop index
 * @param uStop Upper limit for inner loop
 * @return Range2DIterable<T0>
 *
 * @ingroup Util
 */
template <typename T0, typename T1, typename T2, typename T3>
auto range2D(T0 vStart, T1 vStop, T2 uStart, T3 uStop)
{
    return Range2DIterable<T0>(vStart, vStop, uStart, uStop, 1);
}

/**
 * Convenience method for constructing iterable 2D ranges.
 *
 * @param vStart Starting value for outer loop index
 * @param vStop Upper limit for outer loop
 * @param uStart Starting value for inner loop index
 * @param uStop Upper limit for inner loop
 * @param step Step size for both loops
 * @return Range2DIterable<T0>
 *
 * @ingroup Util
 */
template <typename T0, typename T1, typename T2, typename T3, typename T4>
auto range2D(T0 vStart, T1 vStop, T2 uStart, T3 uStop, T4 step)
{
    return Range2DIterable<T0>(vStart, vStop, uStart, uStop, step);
}

// Predeclarations
template <class Iterable>
class EnumerateIterable;

template <class Iterable>
inline auto enumerate(Iterable&& it);

template <typename... Args>
inline EnumerateIterable<std::vector<std::common_type_t<Args...>>> enumerate(
    Args&&... args);

/**
 * Iterable wrapper for enumerating elements of a container by index and value
 *
 * Use volcart::enumerate, which provides a convenient method for constructing
 * an EnumerateIterable.
 *
 * @ingroup Util
 */
template <class Iterable>
class EnumerateIterable
{
private:
    /** enumerate iterator class. Tracks the iterator and index of the item */
    template <class T>
    class EnumerateIterator
    {
    private:
        size_t idx_;
        T it_;

    public:
        /** @{ Iterator type traits */
        using difference_type = size_t;
        using value_type =
            std::pair<const size_t&, decltype(*std::declval<T&>())&>;
        using pointer = value_type*;
        using reference = value_type;
        using iterator_category = std::input_iterator_tag;
        /** @} */

        /** Constructor for the iterator */
        explicit EnumerateIterator(size_t idx, T&& it)
            : idx_{idx}, it_{std::move(it)}
        {
        }

        /** Get the underlying referenced object */
        value_type operator*() const { return {idx_, *it_}; }

        /** Equality comparison: defer to wrapped iterators */
        bool operator==(const EnumerateIterator& other) const
        {
            return it_ == other.it_;
        }

        /** Inequality comparison */
        bool operator!=(const EnumerateIterator& other) const
        {
            return !(*this == other);
        }

        /** Increment the index and the wrapped iterator */
        EnumerateIterator& operator++()
        {
            ++idx_;
            ++it_;
            return *this;
        }
    };

    using IteratorType = decltype(std::begin(std::declval<Iterable>()));
    EnumerateIterable(Iterable&& container)
        : container_{std::forward<Iterable>(container)}
    {
    }

    template <typename... Args>
    EnumerateIterable(Args&&... args) : container_{std::forward<Args>(args)...}
    {
    }

    friend auto enumerate<Iterable>(Iterable&& it);

    template <typename... Args>
    friend EnumerateIterable<std::vector<std::common_type_t<Args...>>>
    enumerate(Args&&... args);

public:
    using iterator = EnumerateIterator<IteratorType>;
    using const_iterator = EnumerateIterator<const IteratorType>;

    /** Get a new EnumerateIterable::iterator */
    iterator begin() { return iterator{0, std::begin(container_)}; }

    /** Get the end-valued EnumerateIterable::iterator */
    iterator end() { return iterator{0, std::end(container_)}; }

    /** Get a new ProgressIterable::const_iterator */
    const_iterator cbegin() const
    {
        return const_iterator{0, std::begin(container_)};
    }

    /** Get the end-valued ProgressIterable::const_iterator */
    const_iterator cend() const
    {
        return const_iterator{0, std::end(container_)};
    }

private:
    Iterable container_;
};

/**
 * @brief Wrap an Iterable into a new one whose iterators return an
 * [index, value] pair
 *
 * This roughly approximates the functionality of Python's enumerate function.
 *
 * @code
 *
 * // Use as std::pair
 * std::vector<int> values{1, 2, 3, 4};
 * for(auto val : enumerate(values)) {
 *     std::cout "values[" << val.first << "] == " << val.second << std::endl;
 * }
 *
 * // Use structured bindings
 * // Update the underlying data
 * for(auto [idx, val] : enumerate(values) {
 *     std::cout << val == values[idx] std::endl;
 *     val++;
 *     std::cout << val == values[idx] std::endl;
 * }
 *
 * @endcode
 *
 * @tparam Iterable Iterable container class. Iterable::iterator must meet
 * LegacyIterator requirements
 * @param it Object of type Iterable
 *
 * @ingroup Util
 */
template <class Iterable>
inline auto enumerate(Iterable&& it)
{
    return EnumerateIterable<Iterable>(std::forward<Iterable>(it));
}

/**
 * @brief Wrap an Iterable into a new one whose iterators return an
 * [index, value] pair
 *
 * This is an overload for lists of arguments:
 *
 * @code
 *
 * for(auto val : enumerate(0, 1, 2, 3)) {
 *     std::cout "values[" << val.first << "] == " << val.second << std::endl;
 * }
 *
 * @endcode
 *
 * @ingroup Util
 */
template <typename... Args>
inline EnumerateIterable<std::vector<std::common_type_t<Args...>>> enumerate(
    Args&&... args)
{
    using Type = std::common_type_t<Args...>;
    using Iterable = std::vector<Type>;
    return EnumerateIterable<Iterable>(std::forward<Args>(args)...);
}

}  // namespace volcart
