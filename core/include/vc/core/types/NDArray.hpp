#pragma once

/** @file */

#include <exception>
#include <functional>
#include <iostream>
#include <iterator>
#include <numeric>
#include <vector>

namespace volcart
{
/**
 * @class NDArray
 * @brief Dynamically-allocated N-Dimensional Array
 *
 * Array is immediately allocated upon construction.
 *
 * Modified from origin project YANDA: https://github.com/csparker247/yanda
 *
 * @ingroup Types
 *
 * @tparam T Type of array elements
 */
template <typename T>
class NDArray
{
public:
    /** Storage container alias */
    using Container = std::vector<T>;
    /** Container index type */
    using IndexType = typename Container::size_type;
    /** Extents type */
    using Extent = std::vector<IndexType>;
    /** N-Dim Array Index type */
    using Index = std::vector<IndexType>;
    /** Iterator type */
    using iterator = typename Container::iterator;
    /** Const iterator type */
    using const_iterator = typename Container::const_iterator;

    /**@{*/
    /** @brief Default constructor */
    explicit NDArray(size_t n) : dim_(n) {}

    /** @brief Constructor with dimensions */
    explicit NDArray(size_t n, Extent e) : dim_(n), extents_(std::move(e))
    {
        if (extents_.size() != dim_) {
            throw std::invalid_argument("Extents of wrong dimension");
        }

        resize_container_();
    }

    /** @overload NDArray(size_t n, Extent e) */
    template <typename... Es>
    explicit NDArray(size_t n, Es... extents)
        : dim_(n), extents_{static_cast<IndexType>(extents)...}
    {
        if (sizeof...(extents) != dim_) {
            throw std::invalid_argument("Extents of wrong dimension");
        }
        resize_container_();
    }

    /** @brief Constructor with range initialization */
    template <typename InputIt>
    explicit NDArray(size_t n, Extent e, InputIt first, InputIt last)
        : dim_(n), extents_(std::move(e)), data_{first, last}
    {
        if (extents_.size() != dim_) {
            throw std::invalid_argument("Extents of wrong dimension");
        }

        auto size = std::accumulate(
            extents_.begin(), extents_.end(), IndexType(1),
            std::multiplies<IndexType>());

        if (size != data_.size()) {
            throw std::invalid_argument(
                "Array extent does not match size of input data");
        }
    }
    /**@}*/

    /**@{*/
    /**
     * @brief Set the extent of the array's dimensions
     *
     * @warning Does not guarantee validity of stored values after resize
     */
    void setExtents(Extent e)
    {
        if (e.size() != dim_) {
            throw std::invalid_argument("Extents of wrong dimension");
        }

        extents_ = std::move(e);
        resize_container_();
    }

    /** @overload void setExtents(Extent e) */
    template <typename... Es>
    void setExtents(Es... extents)
    {
        return setExtents({static_cast<IndexType>(extents)...});
    }

    /** @brief Get the number of dimensions of the array */
    size_t dims() const { return dim_; }

    /** @brief Get the extent (size) of the array's dimensions */
    Extent extents() const { return extents_; }

    /** @brief Get the total number of elements in the array */
    size_t size() const { return data_.size(); }
    /**@}*/

    /**@{*/
    /** @brief Per-element access */
    T& operator()(Index index)
    {

        if (index.size() != dim_) {
            throw std::invalid_argument("Index of wrong dimension");
        }
        return data_.at(index_to_data_index_(index));
    }

    /** @overload T& operator()(Index index) */
    const T& operator()(Index index) const
    {
        if (index.size() != dim_) {
            throw std::invalid_argument("Index of wrong dimension");
        }
        return data_.at(index_to_data_index_(index));
    }

    /** @overload T& operator()(Index index) */
    template <typename... Is>
    T& operator()(Is... indices)
    {
        return operator()(Index{static_cast<IndexType>(indices)...});
    }

    /** @overload T& operator()(Index index) */
    template <typename... Is>
    const T& operator()(Is... indices) const
    {
        return operator()(Index{static_cast<IndexType>(indices)...});
    }

    /** @brief Get slice of array by dropping highest dimension */
    NDArray slice(IndexType index)
    {
        auto offset = std::accumulate(
            std::next(extents_.begin(), 1), extents_.end(), IndexType(1),
            std::multiplies<IndexType>());

        auto b = std::next(data_.begin(), index * offset);
        auto e = std::next(data_.begin(), (index + 1) * offset);

        Extent newExtent{std::next(extents_.begin(), 1), extents_.end()};

        return NDArray(dim_ - 1, newExtent, b, e);
    }
    /**@}*/

    /**@{*/
    /** @brief Return copy of raw data */
    Container as_vector() { return data_; }

    /** @overload as_vector() */
    Container as_vector() const { return data_; }

    /** @brief Get a pointer to the start of the underlying data */
    typename Container::value_type* data() { return data_.data(); }

    /** @overload data() */
    typename Container::value_type* data() const { return data_.data(); }

    /**
     * @brief Return an iterator that points to the first element in the array
     */
    iterator begin() { return std::begin(data_); }

    /** @copydoc begin() */
    const_iterator begin() const { return std::begin(data_); }

    /**
     * @brief Return an iterator that points to the \em past-the-end element
     * in the array
     */
    iterator end() { return std::end(data_); }

    /** @copydoc end() */
    const_iterator end() const { return std::end(data_); }

    /** @brief Return a reference to the first element in the array */
    T& front() { return data_.front(); }

    /** @copydoc front() */
    const T& front() const { return data_.front(); }

    /** @brief Return a reference to the last element in the array */
    T& back() { return data_.back(); }

    /** @copydoc back() */
    const T& back() const { return data_.back(); }
    /**@}*/

    /**
     * @brief Flatten an array by dropping a dimension and appending the
     * data to the next highest dimension
     */
    static void Flatten(NDArray& a, size_t dim)
    {
        if (dim == a.dim_) {
            return;
        } else if (dim > a.dim_) {
            throw std::invalid_argument("Dimension higher than that of array");
        }

        Extent newExtent;

        auto flattened = std::accumulate(
            a.extents_.begin(), std::next(a.extents_.begin(), a.dim_ - dim + 1),
            IndexType(1), std::multiplies<IndexType>());
        newExtent.push_back(flattened);

        newExtent.insert(
            newExtent.end(), std::next(a.extents_.begin(), a.dim_ - dim + 1),
            a.extents_.end());

        a.dim_ = dim;
        a.extents_ = newExtent;
    }

private:
    /** Number of dimensions */
    size_t dim_{1};
    /** Dimension extents */
    Extent extents_;
    /** Data storage */
    Container data_;

    /** Resize the data container to current extents */
    void resize_container_()
    {
        auto size = std::accumulate(
            extents_.begin(), extents_.end(), IndexType(1),
            std::multiplies<IndexType>());

        if (size == 0) {
            throw std::range_error("Array extent is zero");
        }

        data_.resize(size);
    }

    /** Convert item index to data index */
    inline IndexType index_to_data_index_(Index i) const
    {
        IndexType idx{0};
        for (size_t it = 0; it < extents_.size(); it++) {
            auto begin = std::next(extents_.begin(), it + 1);
            auto offset = std::accumulate(
                begin, extents_.end(), IndexType(1),
                std::multiplies<IndexType>());
            idx += i[it] * offset;
        }

        return idx;
    }
};
}  // namespace volcart