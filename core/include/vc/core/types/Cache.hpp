#pragma once

/** @file */

#include <cstddef>

namespace volcart
{
/**
 * @brief Abstract Base Class for Key-Value Caches
 *
 * @tparam TKey Key type
 * @tparam TValue Value type
 */
template <typename TKey, typename TValue>
class Cache
{
public:
    /** Shared Pointer Type */
    using Pointer = std::shared_ptr<Cache<TKey, TValue>>;

    /**@{*/
    /** @brief Set the maximum number of elements in the cache */
    virtual void setCapacity(std::size_t newCapacity) = 0;

    /** @brief Get the maximum number of elements in the cache */
    virtual std::size_t capacity() const = 0;

    /** @brief Get the current number of elements in the cache */
    virtual std::size_t size() const = 0;
    /**@}*/

    /**@{*/
    /** @brief Get an item from the cache by key */
    virtual TValue get(const TKey& k) = 0;

    /** @brief Put an item into the cache */
    virtual void put(const TKey& k, const TValue& v) = 0;

    /** @brief Check if an item is already in the cache */
    virtual bool contains(const TKey& k) = 0;

    /** @brief Clear the cache */
    virtual void purge() = 0;
    /**@}*/

protected:
    /** Default constructor */
    Cache() = default;

    /** Constructor with capacity */
    explicit Cache(std::size_t capacity) : capacity_{capacity} {}

    /** Maximum number of elements in the cache */
    std::size_t capacity_{200};
};
}  // namespace volcart
