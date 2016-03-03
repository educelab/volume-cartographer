#pragma once

#ifndef _VOLCART_LRUCACHE_H_
#define _VOLCART_LRUCACHE_H_

#include <unordered_map>
#include <list>

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
class CacheMissException : public std::exception
{
public:
    virtual const char* what() const noexcept { return "Key not in cache"; }
};

template <typename TKey, typename TValue>
class LRUCache
{
public:
    using TPair = typename std::pair<TKey, TValue>;
    using TListIterator = typename std::list<TPair>::iterator;

    LRUCache() : capacity_(kDefaultCapacity) {}
    LRUCache(const size_t capacity) : capacity_(capacity) {}
    void setCapacity(const size_t newCapacity) { 

        capacity_ = newCapacity;

        // Cleanup elements that exceed the capacity
        while (lookup_.size() > capacity_) {
            auto last = std::end(items_);
            last--;
            lookup_.erase(last->first);
            items_.pop_back();
        }
    }
    
    size_t capacity(void) const { return capacity_; }
    size_t size(void) const { return lookup_.size(); }

    // Returning a const ref is better because then if you try to modify the
    // value without explicitly calling .clone() you'll get a compile error
    const TValue& get(const TKey& k)
    {
        auto lookupIter = lookup_.find(k);
        if (lookupIter == std::end(lookup_)) {
            throw CacheMissException();
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

        if (lookup_.size() > capacity_) {
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
    size_t capacity_;

    static const size_t kDefaultCapacity = 200;
};
}  // namespace volcart

#endif
