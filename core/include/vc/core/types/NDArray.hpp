#pragma once

/** @file */

#include <cstddef>
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
    explicit NDArray(std::size_t n) : dim_(n) {}

    /** @brief Constructor with dimensions */
    explicit NDArray(std::size_t n, Extent e) : dim_(n), extents_(std::move(e))
    {
        if (extents_.size() != dim_) {
            throw std::invalid_argument("Extents of wrong dimension");
        }

        resize_container_();
    }

    /** @overload NDArray(std::size_t n, Extent e) */
    template <typename... Es>
    explicit NDArray(std::size_t n, Es... extents)
        : dim_(n), extents_{static_cast<IndexType>(extents)...}
    {
        if (sizeof...(extents) != dim_) {
            throw std::invalid_argument("Extents of wrong dimension");
        }
        resize_container_();
    }

    /** @brief Constructor with range initialization */
    template <typename InputIt>
    explicit NDArray(std::size_t n, Extent e, InputIt first, InputIt last)
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
    auto dims() const -> std::size_t { return dim_; }

    /** @brief Get the extent (size) of the array's dimensions */
    auto extents() const -> Extent { return extents_; }

    /** @brief Get the total number of elements in the array */
    auto size() const -> std::size_t { return data_.size(); }
    /**@}*/

    /**@{*/
    /** @brief Per-element access */
    auto operator()(Index index) -> T&
    {

        if (index.size() != dim_) {
            throw std::invalid_argument("Index of wrong dimension");
        }
        return data_.at(index_to_data_index_(index));
    }

    /** @overload T& operator()(Index index) */
    auto operator()(Index index) const -> const T&
    {
        if (index.size() != dim_) {
            throw std::invalid_argument("Index of wrong dimension");
        }
        return data_.at(index_to_data_index_(index));
    }

    /** @overload T& operator()(Index index) */
    template <typename... Is>
    auto operator()(Is... indices) -> T&
    {
        return operator()(Index{static_cast<IndexType>(indices)...});
    }

    /** @overload T& operator()(Index index) */
    template <typename... Is>
    auto operator()(Is... indices) const -> const T&
    {
        return operator()(Index{static_cast<IndexType>(indices)...});
    }

    /** @brief Get slice of array by dropping highest dimension */
    auto slice(IndexType index) -> NDArray
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
    auto as_vector() -> Container { return data_; }

    /** @overload as_vector() */
    auto as_vector() const -> Container { return data_; }

    /** @brief Get a pointer to the start of the underlying data */
    auto data() -> typename Container::value_type* { return data_.data(); }

    /** @overload data() */
    auto data() const -> typename Container::value_type*
    {
        return data_.data();
    }

    /**
     * @brief Return an iterator that points to the first element in the array
     */
    auto begin() -> iterator { return std::begin(data_); }

    /** @copydoc begin() */
    auto begin() const -> const_iterator { return std::begin(data_); }

    /**
     * @brief Return an iterator that points to the \em past-the-end element
     * in the array
     */
    auto end() -> iterator { return std::end(data_); }

    /** @copydoc end() */
    auto end() const -> const_iterator { return std::end(data_); }

    /** @brief Return a reference to the first element in the array */
    auto front() -> T& { return data_.front(); }

    /** @copydoc front() */
    auto front() const -> const T& { return data_.front(); }

    /** @brief Return a reference to the last element in the array */
    auto back() -> T& { return data_.back(); }

    /** @copydoc back() */
    auto back() const -> const T& { return data_.back(); }
    /**@}*/

    /**
     * @brief Flatten an array by dropping a dimension and appending the
     * data to the next highest dimension
     */
    static void Flatten(NDArray& a, std::size_t dim)
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
    std::size_t dim_{1};
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
    inline auto index_to_data_index_(Index i) const -> IndexType
    {
        IndexType idx{0};
        for (std::size_t it = 0; it < extents_.size(); it++) {
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