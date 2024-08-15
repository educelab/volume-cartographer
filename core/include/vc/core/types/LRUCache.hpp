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
 * Data elements are stored in a std::list, ordered from most to least recently
 * used. A key-value map into this list is stored in a std::unordered_map.
 *
 * If a function is provided to onEvict(), cache entries are only removed if
 * the function returns `true` for the given entry. See the documentation of
 * onEvict() for more details.
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
        evict();
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
            auto& [key, val] = *lookupIter->second;
            if (BaseClass::on_eject_) {
                BaseClass::on_eject_(key, val);
            }
            items_.erase(lookupIter->second);
            lookup_.erase(lookupIter);
        }

        items_.push_front(TPair(k, v));
        lookup_[k] = std::begin(items_);
        evict();
    }

    /** @brief Check if an item is already in the cache */
    auto contains(const TKey& k) -> bool override
    {
        std::shared_lock lock(cache_mutex_);
        return lookup_.find(k) != std::end(lookup_);
    }

    /**
     * @brief Set a callback function for validating if an item can be evicted
     *
     * This optional function is called whenever a cache entry is about to be
     * evicted. Minimally, it should return `true` if it is safe to remove the
     * cache entry and `false` otherwise. Optionally, it may also perform
     * cleanup operations for the stored item, such as manual deallocation.
     * Entries are only tested for eviction when calling setCapacity(), put(),
     * evict(), and purge().
     *
     * @note Depending upon the function provided, the cache may temporarily
     * store more entries than the capacity suggests.
     */
    void onEvict(std::function<bool(TKey&, TValue&)> fn) override
    {
        BaseClass::on_eject_ = fn;
    }

    /**
     * @brief Remove the validation callback function
     *
     * @see onEvict()
     */
    void resetOnEvict() override { BaseClass::on_eject_ = {}; }

    /**
     * @brief Evict items following the cache policy
     *
     * Automatically called whenever the cache size is expected to exceed the
     * current capacity. Normally should not need to be called directly except
     * when using onEvict() to conditionally keep items which would otherwise
     * be evicted.
     */
    void evict() override
    {  // Already below capacity
        if (lookup_.size() <= capacity_) {
            return;
        }

        // If we don't have an onEject callback, fast remove the last element
        if (not BaseClass::on_eject_) {
            while (lookup_.size() > capacity_) {
                auto& [key, _] = items_.back();
                lookup_.erase(key);
                items_.pop_back();
            }
            return;
        }

        // Else iterate the items in reverse until we find one we can remove
        for (auto it = items_.rbegin(); it != items_.rend();) {
            // Get the key and value
            auto& [key, value] = *it;

            // Eject this item
            if (BaseClass::on_eject_(key, value)) {
                lookup_.erase(key);
                it = std::next(it);
                it = std::reverse_iterator(items_.erase(it.base()));
            } else {
                ++it;
            }

            // Stop when we've reset to capacity
            if (lookup_.size() <= capacity_) {
                break;
            }
        }
    }

    /** @brief Evict all items ignoring cache policy */
    void purge() override
    {
        std::unique_lock lock(cache_mutex_);
        // If we have an on_eject_ function, check every item
        if (BaseClass::on_eject_) {
            for (auto it = items_.begin(); it != items_.end();) {
                auto& [key, value] = *it;
                if (BaseClass::on_eject_(key, value)) {
                    lookup_.erase(key);
                    it = items_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        // Otherwise, remove everything
        else {
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
};
}  // namespace volcart
