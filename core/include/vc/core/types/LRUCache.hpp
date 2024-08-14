#pragma once

/** @file */

#include <cstddef>
#include <list>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

#include "vc/core/types/Cache.hpp"

namespace volcart
{

/** No-op mutex */
struct NoOpMutex {
};

/**
 * @class LRUCache
 * @brief Least Recently Used Cache
 * @author Sean Karlage
 *
 * A cache using a least recently used replacement policy. As elements are used,
 * they are moved to the front of the cache. When the cache exceeds capacity,
 * elements are popped from the end of the cache and replacement elements are
 * added to the front.
 *
 * Usage information is tracked in a std::unordered_map since its performance
 * will likely be the fastest of the STL classes. This should be profiled. Data
 * elements are stored in insertion order in a std::list and are pointed to
 * by the elements in the usage map.
 *
 * Design mostly taken from
 * <a href = "https://github.com/lamerman/cpp-lru-cache">here</a>.
 *
 * @ingroup Types
 */
template <typename TKey, typename TValue, class TMutex = std::shared_mutex>
class LRUCache final : public Cache<TKey, TValue>
{
public:
    using BaseClass = Cache<TKey, TValue>;
    using BaseClass::capacity_;

    /**
     * @brief Templated Key/Value pair
     *
     * Stored in the data list.
     */
    using TPair = std::pair<TKey, TValue>;

    /**
     * @brief Templated Key/Value pair iterator
     *
     * Stored in the LRU map.
     */
    using TListIterator = typename std::list<TPair>::iterator;

    /** Shared pointer type */
    using Pointer = std::shared_ptr<LRUCache<TKey, TValue>>;

    /**@{*/
    /** @brief Default constructor */
    LRUCache() : BaseClass() {}

    /** @brief Constructor with cache capacity parameter */
    explicit LRUCache(std::size_t capacity) : BaseClass(capacity) {}

    /** @overload LRUCache() */
    static auto New() -> Pointer
    {
        return std::make_shared<LRUCache<TKey, TValue>>();
    }

    /** @overload LRUCache(std::size_t) */
    static auto New(std::size_t capacity) -> Pointer
    {
        return std::make_shared<LRUCache<TKey, TValue>>(capacity);
    }
    /**@}*/

    /**@{*/
    /** @brief Set the maximum number of elements in the cache */
    void setCapacity(std::size_t capacity) override
    {
        std::unique_lock lock(cache_mutex_);
        if (capacity <= 0) {
            throw std::invalid_argument(
                "Cannot create cache with capacity <= 0");
        }
        capacity_ = capacity;
        ejectToCapacity_();
    }

    /** @brief Get the maximum number of elements in the cache */
    auto capacity() const -> std::size_t override { return capacity_; }

    /** @brief Get the current number of elements in the cache */
    auto size() const -> std::size_t override { return lookup_.size(); }
    /**@}*/

    /**@{*/
    /** @brief Get an item from the cache by key */
    auto get(const TKey& k) -> TValue override
    {
        std::unique_lock lock(cache_mutex_);
        auto lookupIter = lookup_.find(k);
        if (lookupIter == std::end(lookup_)) {
            throw std::invalid_argument("Key not in cache");
        }

        items_.splice(std::begin(items_), items_, lookupIter->second);
        return lookupIter->second->second;
    }

    /** @brief Put an item into the cache */
    void put(const TKey& k, const TValue& v) override
    {
        std::unique_lock lock(cache_mutex_);
        // If already in cache, need to refresh it
        auto lookupIter = lookup_.find(k);
        if (lookupIter != std::end(lookup_)) {
            items_.erase(lookupIter->second);
            lookup_.erase(lookupIter);
        }

        items_.push_front(TPair(k, v));
        lookup_[k] = std::begin(items_);
        ejectToCapacity_();
    }

    /** @brief Check if an item is already in the cache */
    auto contains(const TKey& k) -> bool override
    {
        std::shared_lock lock(cache_mutex_);
        return lookup_.find(k) != std::end(lookup_);
    }

    /** @brief Clear the cache */
    void purge() override
    {
        std::unique_lock lock(cache_mutex_);
        // If we have an on_eject_ function, check every item
        std::size_t cnt{0};
        if (BaseClass::on_eject_) {
            for (auto& [key, value] : items_) {
                if (BaseClass::on_eject_(key, value)) {
                    lookup_.erase(key);
                    items_.pop_front();
                    ++cnt;
                }
            }
        }

        // Otherwise, remove everything
        else {
            cnt = items_.size();
            lookup_.clear();
            items_.clear();
        }
    }
    /**@}*/

private:
    /** Cache data storage */
    std::list<TPair> items_;
    /** Cache usage information */
    std::unordered_map<TKey, TListIterator> lookup_;
    /** Shared mutex for thread-safe access */
    mutable TMutex cache_mutex_;
    /** Eject items until capacity is reached */
    void ejectToCapacity_()
    {
        // Count of ejected items
        std::size_t cnt{0};
        for (auto it = items_.rbegin(); it != items_.rend();) {
            // Check if this item can be ejected
            auto& [key, value] = *it;
            bool canEject{true};
            if (BaseClass::on_eject_) {
                canEject = BaseClass::on_eject_(key, value);
            }

            // Eject this item
            if (canEject) {
                lookup_.erase(key);
                it = std::next(it);
                items_.erase(it.base());  // TODO: Verify this
                ++cnt;
            } else {
                ++it;
            }

            // Stop when we've reset to capacity
            if (lookup_.size() < capacity_) {
                break;
            }
        }
    }
};
}  // namespace volcart
