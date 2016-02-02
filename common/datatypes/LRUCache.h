#pragma once

#ifndef _VC_LRUCACHE_H_
#define _VC_LRUCACHE_H_

#include <unordered_map>
#include <list>
#include <iostream>
#include <chrono>

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

    LRUCache() : size_(kDefaultSize) {}
    LRUCache(const size_t size) : size_(size) {}
    void setSize(const size_t newSize) { size_ = newSize; }
    size_t size(void) const { return size_; }
    // Need to find a better way than returning raw pointer --> could totally
    // use std::optional if it wasn't still experimental...
    const TValue* get(const TKey& k)
    {
        auto lookupIter = lookup_.find(k);
        if (lookupIter == std::end(lookup_)) {
            return nullptr;
        } else {
            items_.splice(std::begin(items_), items_, lookupIter->second);
            return &lookupIter->second->second;
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

        if (lookup_.size() > size_) {
            auto last = std::end(items_);
            last--;
            lookup_.erase(last->first);
            items_.pop_back();
        }
    }

    bool exists(const TKey& k) { return lookup_.find(k) != std::end(lookup_); }
private:
    std::list<TPair> items_;
    std::unordered_map<TKey, TListIterator> lookup_;
    size_t size_;

    static const size_t kDefaultSize = 200;
};

}  // namespace volcart

#endif  // _VC_LRUCACHE_H_
