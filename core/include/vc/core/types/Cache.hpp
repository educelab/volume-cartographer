#pragma once

/** @file */

#include <cstddef>
#include <functional>

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
    /** Default destructor */
    virtual ~Cache() = default;

    /** Shared Pointer Type */
    using Pointer = std::shared_ptr<Cache>;

    /**@{*/
    /** @brief Set the maximum number of elements in the cache */
    virtual void setCapacity(std::size_t newCapacity) = 0;

    /** @brief Get the maximum number of elements in the cache */
    [[nodiscard]] virtual auto capacity() const -> std::size_t = 0;

    /** @brief Get the current number of elements in the cache */
    [[nodiscard]] virtual auto size() const -> std::size_t = 0;
    /**@}*/

    /**@{*/
    /** @brief Get an item from the cache by key */
    virtual auto get(const TKey& k) -> TValue = 0;

    /** @brief Put an item into the cache */
    virtual auto put(const TKey& k, const TValue& v) -> void = 0;

    /** @brief Check if an item is already in the cache */
    virtual auto contains(const TKey& k) -> bool = 0;

    /** @brief Callback function when items are ejected */
    virtual void onEject(std::function<bool(TKey&, TValue&)> fn) = 0;

    /** @brief Clear the cache */
    virtual auto purge() -> void = 0;
    /**@}*/

protected:
    /** Default constructor */
    Cache() = default;

    /** Constructor with capacity */
    explicit Cache(const std::size_t capacity) : capacity_{capacity} {}

    /** Maximum number of elements in the cache */
    std::size_t capacity_{200};

    /** Callback to verify if an entry can be ejected */
    std::function<bool(TKey&, TValue&)> on_eject_;
};
}  // namespace volcart
