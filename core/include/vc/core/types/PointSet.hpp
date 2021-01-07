#pragma once

#include <algorithm>
#include <cassert>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace volcart
{
/**
 * @class PointSet
 * @brief Holds a collection of points
 * @author Sean Karlage
 *
 * @ingroup Types
 */
template <typename T>
class PointSet
{
public:
    /** Point type */
    using Point = T;

    /** Iterator value type */
    using value_type = Point;

    /** Container type. Defines the underlying data structure. Should supply an
     * STL compliant interface. */
    using Container = std::vector<Point>;

    /** Iterator type */
    using Iterator = typename Container::iterator;

    /** Const iterator type */
    using ConstIterator = typename Container::const_iterator;

    /** PointSet version number */
    constexpr static int FORMAT_VERSION = 1;

    /** PointSet header terminator */
    constexpr static auto HEADER_TERMINATOR = "<>";

    /** Regex for header terminator */
    constexpr static auto HEADER_TERMINATOR_REGEX = "^<>$";

    /** Pointer type */
    using Pointer = std::shared_ptr<PointSet<T>>;

    /**@{*/
    /** @brief Default constructor */
    explicit PointSet() = default;

    /** @brief Construct and preallocate a number of Point elements */
    explicit PointSet(size_t initSize) { data_.reserve(initSize); }

    /** @brief Construct and fill a number of elements with an initial value */
    explicit PointSet(size_t initSize, T initVal)
    {
        data_.assign(initSize, initVal);
    }
    /**@}*/

    /**@{*/
    /** @brief Get a Point by index */
    const T& operator[](size_t idx) const
    {
        assert(idx < data_.size() && "idx out of range");
        return data_[idx];
    }

    /** @copydoc operator[]() */
    T& operator[](size_t idx)
    {
        assert(idx < data_.size() && "idx out of range");
        return data_[idx];
    }
    /**@}*/

    /**@{*/
    /** @brief Get the size of the PointSet */
    size_t size() const { return data_.size(); }

    /** @brief Return whether the PointSet is empty */
    bool empty() const { return data_.empty(); }

    /** @brief Get the PointSet storage container */
    Container as_vector() { return data_; }

    /** @brief Remove all elements from the PointSet */
    void clear() { data_.clear(); }
    /**@}*/

    /**@{*/
    /** @brief Add a Point to the PointSet */
    void push_back(const T& val) { data_.push_back(val); }

    /** @overload push_back() */
    void push_back(T&& val) { data_.push_back(val); }

    /** @brief Emplace a Point at the back of the PointSet */
    template <class... Args>
    void emplace_back(Args&&... args)
    {
        data_.emplace_back(std::forward<Args>(args)...);
    }

    /** @brief Append a PointSet to the end of the current one */
    template <class ContainerType>
    void append(const ContainerType& c)
    {
        std::copy(std::begin(c), std::end(c), std::back_inserter(data_));
    }
    /**@}*/

    /**@{*/
    /** @brief Return an iterator that points to the first element in the
     * PointSet */
    Iterator begin() { return std::begin(data_); }

    /** @copydoc begin() */
    ConstIterator begin() const { return std::begin(data_); }

    /** @brief Return an iterator that points to the \em past-the-end element
     * in the PointSet */
    Iterator end() { return std::end(data_); }

    /** @copydoc end() */
    ConstIterator end() const { return std::end(data_); }

    /** @brief Return a reference to the first element in the PointSet */
    T& front() { return data_.front(); }

    /** @copydoc front() */
    const T& front() const { return data_.front(); }

    /** @brief Return a reference to the last element in the PointSet */
    T& back() { return data_.back(); }

    /** @copydoc back() */
    const T& back() const { return data_.back(); }
    /**@}*/

    /**@{*/
    /** @brief Return the element with the smallest absolute norm (L2)
     *
     * @warning Point type must be convertible to an OpenCV `cv::_InputArray`.
     */
    T min() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::min_element(
            std::begin(data_), std::end(data_),
            [](auto lhs, auto rhs) { return cv::norm(lhs) < cv::norm(rhs); });
    }

    /** @brief Return the element with the largest absolute norm (L2)
     *
     * @copydetails min()
     */
    T max() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        return *std::max_element(
            std::begin(data_), std::end(data_),
            [](auto lhs, auto rhs) { return cv::norm(lhs) < cv::norm(rhs); });
    }

    /** @brief Return a pair of elements containing the points with the smallest
     * and largest absolute norm (L2)
     *
     * @copydetails min()
     */
    std::pair<T, T> minMax() const
    {
        if (empty()) {
            throw std::range_error("empty PointSet");
        }
        auto pair = std::minmax_element(
            std::begin(data_), std::end(data_),
            [](auto lhs, auto rhs) { return cv::norm(lhs) < cv::norm(rhs); });
        return {*pair.first, *pair.second};
    }
    /**@}*/

    /**@{*/
    /** @brief Create a PointSet of a specific size, filled with an initial
     * value */
    static PointSet Fill(size_t initSize, T initVal)
    {
        return PointSet(initSize, initVal);
    }
    /**@}*/

protected:
    /** Point container */
    Container data_;
};
}  // namespace volcart
