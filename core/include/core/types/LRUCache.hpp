#pragma once

#include <list>
#include <unordered_map>

/*
 * A cache of objects implementing the least-recently-used algorithm
 * for eviction. You can tweak the number of entries you want in the cache.
 * Uses std::unordered_map since it's performance will likely be fastest. If
 * necessary, this should probably be profiled
 *
 * Design mostly taken from here:
 * https://github.com/lamerman/cpp-lru-cache/blob/master/include/lrucache.hpp
 */

namespace volcart
{

template <typename TKey, typename TValue>
class LRUCache
{
public:
    using TPair = typename std::pair<TKey, TValue>;
    using TListIterator = typename std::list<TPair>::iterator;

    LRUCache() : capacity_{DEFAULT_CAPACITY} {}
    explicit LRUCache(int64_t capacity) : capacity_{capacity} {}

    void setCapacity(int64_t newCapacity)
    {
        if (newCapacity <= 0) {
            throw std::invalid_argument(
                "Cannot create cache with capacity <= 0");
        } else {
            capacity_ = newCapacity;
        }

        // Cleanup elements that exceed the capacity
        while (int64_t(lookup_.size()) > capacity_) {
            auto last = std::end(items_);
            last--;
            lookup_.erase(last->first);
            items_.pop_back();
        }
    }

    int64_t capacity() const { return capacity_; }
    size_t size() const { return lookup_.size(); }

    // Returning a const ref is better because then if you try to modify the
    // value without explicitly calling .clone() you'll get a compile error
    const TValue& get(const TKey& k)
    {
        auto lookupIter = lookup_.find(k);
        if (lookupIter == std::end(lookup_)) {
            throw std::invalid_argument("Key not in cache");
        } else {
            items_.splice(std::begin(items_), items_, lookupIter->second);
            return lookupIter->second->second;
        }
    }

    void put(const TKey& k, const TValue& v)
    {
        // If already in cache, need to refresh it
        auto lookupIter = lookup_.find(k);
        if (lookupIter != std::end(lookup_)) {
            items_.erase(lookupIter->second);
            lookup_.erase(lookupIter);
        }

        items_.push_front(TPair(k, v));
        lookup_[k] = std::begin(items_);

        if (int64_t(lookup_.size()) > capacity_) {
            auto last = std::end(items_);
            last--;
            lookup_.erase(last->first);
            items_.pop_back();
        }
    }

    void purge()
    {
        lookup_.clear();
        items_.clear();
    }

    bool exists(const TKey& k) { return lookup_.find(k) != std::end(lookup_); }

private:
    std::list<TPair> items_;
    std::unordered_map<TKey, TListIterator> lookup_;
    int64_t capacity_;

    static constexpr int64_t DEFAULT_CAPACITY = 200;
};
}  // namespace volcart
