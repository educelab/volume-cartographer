#include <cstddef>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "vc/core/types/LRUCache.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;

///// FIXTURES /////
class LRUCache_Empty : public testing::Test
{
public:
    LRUCache<std::size_t, int> cache;
};

class LRUCache_Filled : public LRUCache_Empty
{
public:
    LRUCache_Filled()
    {
        // change capacity to 100
        cache.setCapacity(100);

        // fill cache with key ordered from 0-99 and values equaling to key^2
        // cache will push new pairs to front when adding, so final list order
        // will be 99,98,...,0
        for (std::size_t idx = 0; idx < cache.capacity(); idx++) {

            // set the cache item pairs
            cache.put(idx, (idx * idx));
        }
    }
};

///// TEST CASES /////
// checks the original capacity of 200 and that the item list is empty
// then changes capacity and check the new capacity and that the list
// is still empty
TEST_F(LRUCache_Empty, ResizeCapacity)
{
    // Defaults
    EXPECT_EQ(cache.capacity(), 200);
    EXPECT_EQ(cache.size(), 0);

    // Changed
    cache.setCapacity(50);
    EXPECT_EQ(cache.capacity(), 50);
    EXPECT_EQ(cache.size(), 0);
}

// checks that the capacity and size are equal and that the values are correct
// then checks that referencing a negative key is caught
// then checks that referencing a key greater that the cache size is caught
TEST_F(LRUCache_Filled, CheckReferenceToOutOfBoundsKey)
{
    EXPECT_EQ(cache.capacity(), cache.size());
    for (std::size_t key = 0; key < cache.capacity(); key++) {
        EXPECT_EQ(cache.get(key), key * key);
    }

    auto idx = cache.capacity() - (cache.capacity() + 1);
    try {
        EXPECT_ANY_THROW(cache.get(idx));
    } catch (std::exception& e) {
        // exception output message
        // casting negative size_t value to int
        Logger()->debug("{}", e.what());
        Logger()->debug("Key Tried: {}", idx);
        EXPECT_TRUE(true);
    }

    try {
        EXPECT_ANY_THROW(cache.get(cache.capacity()));
    } catch (std::exception& e) {
        Logger()->debug("{}", e.what());
        Logger()->debug("Key Tried: {}", cache.capacity());
    }
}

// checks that capacity and size equal 100 then inserts an item pair
// checks that key == 100 exists and that capacity and size are unchanged
// checks that item key == 0 was popped and the newly added pair is accessible
TEST_F(LRUCache_Filled, InsertIntoCacheAndConfirmExistence)
{
    EXPECT_EQ(cache.capacity(), cache.size());
    EXPECT_EQ(cache.capacity(), 100);

    cache.put(cache.capacity(), (cache.capacity() * cache.capacity()));

    EXPECT_TRUE(cache.contains(100));
    EXPECT_EQ(cache.capacity(), 100);
    EXPECT_EQ(cache.size(), 100);
    EXPECT_EQ(cache.contains(0), false);
    EXPECT_EQ(cache.get(100), 10000);
}

TEST_F(LRUCache_Empty, CheckCacheWithDifferingCapacityAndSize)
{
    // checks that initial capacity is 200 with size 0 then fills the item list
    // halfway and checks that the size is now 100 and capacity is unchanged
    EXPECT_EQ(cache.capacity(), 200);
    EXPECT_EQ(cache.size(), 0);

    for (std::size_t key = 0; key < cache.capacity(); key += 2) {
        // add even numbers, zero inclusive, and double their key as the value
        // will add items in descending key value order --> 198,196...,2,0
        cache.put(key, key + key);
    }

    EXPECT_EQ(cache.capacity(), 200);
    EXPECT_EQ(cache.size(), 100);

    // test to see what happens when referencing an odd-number key
    // also, checking if oddKey exists --> redundant
    for (std::size_t oddKey = 1; oddKey < cache.capacity(); oddKey += 2) {
        try {
            EXPECT_EQ(cache.contains(oddKey), false);
            EXPECT_ANY_THROW(cache.get(oddKey));
        } catch (std::exception& e) {
            EXPECT_TRUE(true);
        }
    }
    // update capacity to size 50, which should pop items that are at the 'end'
    // of the cache, thus, we'll check item key 98,96,...,2,0
    cache.setCapacity(50);

    for (std::size_t k = 0; k < cache.capacity() * 2; k += 2) {
        try {
            EXPECT_EQ(cache.contains(k), false);
            EXPECT_ANY_THROW(cache.get(k));
        } catch (std::exception& e) {
            EXPECT_TRUE(true);
        }
    }

    // check that keys 100 and 198 still exist
    EXPECT_TRUE(cache.contains(100));
    EXPECT_TRUE(cache.contains(198));
}

// checks that cache has initial capacity of 200 and size 0, then fills item
// list 1 past capacity and checks that capacity is still 200 and size is 200
// then checks that item key 0 should be popped with the last iteration of for
// loop
TEST_F(LRUCache_Empty, TryToInsertMorePairsThanCurrentCapacity)
{
    EXPECT_EQ(cache.capacity(), 200);
    EXPECT_EQ(cache.size(), 0);

    for (std::size_t key = 0; key <= cache.capacity(); key++) {
        cache.put(key, key + key);
    }

    EXPECT_EQ(cache.capacity(), 200);
    EXPECT_EQ(cache.size(), 200);
    EXPECT_EQ(cache.contains(0), false);
}

// checks that setting a negative cap cache fails, then checks
// that capacity and size remain unchanged from initial values
TEST_F(LRUCache_Empty, TryToInsertIntoZeroCapacityCache)
{
    try {
        EXPECT_ANY_THROW(cache.setCapacity(0));
    } catch (std::exception& e) {
        Logger()->debug("{}", e.what());
    }

    EXPECT_EQ(cache.capacity(), 200);
    EXPECT_EQ(cache.size(), 0);
}

// checks that capacity and size are equal then after puring size is still 0
// checks that item key 1 does not exist (implied by 0 size)
TEST_F(LRUCache_Filled, PurgeTheCache)
{
    EXPECT_EQ(cache.capacity(), cache.size());
    cache.purge();
    EXPECT_EQ(cache.size(), 0);
    EXPECT_EQ(cache.contains(1), false);
}

TEST(LRUCache, OnEvict)
{
    // On evict test function:
    // Returns true if the Mat doesn't point to anything OR the refcount is <= 1
    // (i.e. the cache entry is the only remaining ref)
    auto onEvict = [](int& key, cv::Mat& i) -> bool {
        return not i.u or i.u->refcount <= 1;
    };

    // Set up cache
    LRUCache<int, cv::Mat> cache;
    cache.setCapacity(10);
    cache.onEvict(onEvict);

    // Create some images
    std::vector<cv::Mat> imgs;
    for (auto key : range(10)) {
        cv::Mat img = cv::Mat::zeros(10, 10, CV_8UC1);
        imgs.push_back(img);
        cache.put(key, img);
    }

    // Check sizes
    EXPECT_EQ(cache.capacity(), 10);
    EXPECT_EQ(cache.size(), 10);

    // Change capacity
    cache.setCapacity(9);

    // Shouldn't have changed (all keys still have reference)
    EXPECT_EQ(cache.capacity(), 9);
    EXPECT_EQ(cache.size(), 10);

    // Try eviction
    cache.evict();

    // Shouldn't have changed (all keys still have reference)
    EXPECT_EQ(cache.capacity(), 9);
    EXPECT_EQ(cache.size(), 10);

    // Remove reference to one of the Mats and evict the cache
    imgs.erase(imgs.begin());
    cache.evict();

    // Should have changed
    EXPECT_EQ(cache.capacity(), 9);
    EXPECT_EQ(cache.size(), 9);
    EXPECT_FALSE(cache.contains(0));
    EXPECT_ANY_THROW(cache.get(0));
    for (const auto key : range(1, 10)) {
        EXPECT_TRUE(cache.contains(key));
        EXPECT_NO_THROW(cache.get(key));
    }

    // Remove references to some Mats and purge the cache
    imgs.erase(imgs.begin(), std::next(imgs.begin(), 3));
    cache.purge();

    // Should still have items that don't pass test
    EXPECT_EQ(cache.capacity(), 9);
    EXPECT_EQ(cache.size(), 6);

    // Remove references to all Mats and purge the cache
    imgs.clear();
    cache.purge();

    // Should still have items that don't pass test
    EXPECT_EQ(cache.capacity(), 9);
    EXPECT_EQ(cache.size(), 0);
}