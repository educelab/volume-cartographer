//
// Created by Ryan Taber on 2/29/16.
//

#define BOOST_TEST_MODULE LRUCache

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "vc/core/types/LRUCache.hpp"

/************************************************************************************
 *                                                                                  *
 *  LRUCacheTest.cpp - tests the functionality of
 * /v-c/core/datatypes/LRUCache.h
 *  The ultimate goal of this file is the following: *
 *                                                                                  *
 *  Check that we are able to create a cache, retrieve/set key-value pairs *
 *  appropriately, and handle exceptions *
 *                                                                                  *
 *  This file is broken up into testing fixtures which initialize the *
 *  objects used in each of the three test cases. *
 *                                                                                  *
 *  1. CheckAbilityToResizeCache *
 *  2. CheckReferenceToOutOfBoundsKey *
 *  3. InsertIntoCacheAndConfirmExistence *
 *  4. CheckCacheWithDifferingCapacityAndSize *
 *  5. TryToInsertMorePairsThanCurrentCapacity *
 *  6. PurgeTheCache *
 *                                                                                  *
 * Input: *
 *     No required inputs for this sample test. *
 *                                                                                  *
 * Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general  *
 *     number of testing errors is output. *
 *                                                                                  *
 * **********************************************************************************/

/***************
 *
 * FIXTURES
 *
 ***************/

struct CreateCacheWithDefaultConstructorFixture {

    CreateCacheWithDefaultConstructorFixture()
    {

        std::cerr << "Creating cache with empty constructor..." << std::endl;
    }

    ~CreateCacheWithDefaultConstructorFixture()
    {
        std::cerr << "Destroying cache..." << std::endl;
    }

    volcart::LRUCache<size_t, int> _DefaultCache;
};

struct ResizingCacheFixture {

    ResizingCacheFixture()
    {

        std::cerr << "Creating cache that will be resized..." << std::endl;

        _in_NewCap = 50;
    }
    ~ResizingCacheFixture() { std::cerr << "Destroying cache..." << std::endl; }

    volcart::LRUCache<size_t, int> _DefaultCache;
    size_t _in_NewCap;
};

struct ReferenceBadKeyFixture {

    ReferenceBadKeyFixture()
    {

        std::cerr << "Creating cache..." << std::endl;

        // change capacity to 100
        _Cache.setCapacity(100);

        // fill cache with key ordered from 0-99 and values equaling to key^2
        // cache will push new pairs to front when adding, so final list order
        // will be 99,98,...,0
        for (_in_Cap = 0; _in_Cap < _Cache.capacity(); _in_Cap++) {

            // set the cache item pairs
            _Cache.put(_in_Cap, (_in_Cap * _in_Cap));
        }
    }
    ~ReferenceBadKeyFixture()
    {
        std::cerr << "Destroying cache..." << std::endl;
    }

    volcart::LRUCache<size_t, int> _Cache;
    size_t _in_Cap;
};

/***************
 *
 * TEST CASES
 *
 ***************/

BOOST_FIXTURE_TEST_CASE(CheckAbilityToResizeCache, ResizingCacheFixture)
{

    BOOST_CHECK_EQUAL(
        _DefaultCache.capacity(), 200);  // check original capacity of 200
    BOOST_CHECK_EQUAL(_DefaultCache.size(), 0);       // check that item list is
                                                      // empty
    _DefaultCache.setCapacity(_in_NewCap);            // change capacity
    BOOST_CHECK_EQUAL(_DefaultCache.capacity(), 50);  // check new capacity
    BOOST_CHECK_EQUAL(
        _DefaultCache.size(), 0);  // check that item list is still empty
}

BOOST_FIXTURE_TEST_CASE(CheckReferenceToOutOfBoundsKey, ReferenceBadKeyFixture)
{

    BOOST_CHECK_EQUAL(
        _Cache.capacity(), _Cache.size());  // capacity and size should be equal

    for (size_t key = 0; key < _Cache.capacity(); key++) {

        BOOST_CHECK_EQUAL(
            _Cache.get(key), key * key);  // check that values are correct
    }

    try {
        _Cache.get(
            _Cache.capacity() -
            (_Cache.capacity() + 1));  // try to reference key < 0
        BOOST_CHECK(false);  // return failed test if exception not caught
    } catch (std::exception& e) {

        // exception output message
        // casting negative size_t value to int
        std::cout << e.what() << std::endl;
        std::cout << "Key Tried: "
                  << static_cast<int>(
                         _Cache.capacity() - (_Cache.capacity() + 1))
                  << std::endl;
        BOOST_CHECK(true);
    }

    try {
        _Cache.get(_Cache.capacity());  // try to reference key > cache size
        BOOST_CHECK(false);  // return failed test if exception not caught
    } catch (std::exception& e) {

        std::cout << e.what() << std::endl;
        std::cout << "Key Tried: " << _Cache.capacity() << std::endl;
        BOOST_CHECK(true);
    }
}

BOOST_FIXTURE_TEST_CASE(
    InsertIntoCacheAndConfirmExistence, ReferenceBadKeyFixture)
{

    BOOST_CHECK_EQUAL(
        _Cache.capacity(), _Cache.size());  // check capacity and size are equal
    BOOST_CHECK_EQUAL(
        _Cache.capacity(), 100);  // check cap is 100 (size implied)

    _Cache.put(
        _Cache.capacity(),
        (_Cache.capacity() * _Cache.capacity()));  // insert item pair

    BOOST_CHECK(_Cache.exists(100));  // check that key == 100 exists now

    BOOST_CHECK_EQUAL(
        _Cache.capacity(), 100);  // capacity and size should be unchanged
    BOOST_CHECK_EQUAL(_Cache.size(), 100);

    BOOST_CHECK_EQUAL(
        _Cache.exists(0), false);  // check that item key == 0 was popped

    BOOST_CHECK_EQUAL(
        _Cache.get(100), 10000);  // check that we can access newly added pair
}

BOOST_FIXTURE_TEST_CASE(
    CheckCacheWithDifferingCapacityAndSize,
    CreateCacheWithDefaultConstructorFixture)
{

    BOOST_CHECK_EQUAL(
        _DefaultCache.capacity(),
        200);  // cache should have initial capacity of 200
    BOOST_CHECK_EQUAL(_DefaultCache.size(), 0);  // size should be 0

    for (size_t key = 0; key < _DefaultCache.capacity();
         key += 2) {  // fill item list halfway

        // add even numbers, zero inclusive, and double their key as the value
        // will add items in descending key value order --> 198,196...,2,0
        _DefaultCache.put(key, key + key);
    }

    BOOST_CHECK_EQUAL(
        _DefaultCache.capacity(),
        200);  // now size should be 100 and capacity still 200
    BOOST_CHECK_EQUAL(_DefaultCache.size(), 100);

    // test to see what happens when referencing an odd-number key
    // also, checking if oddKey exists --> redundant
    for (size_t oddKey = 1; oddKey < _DefaultCache.capacity(); oddKey += 2) {

        try {

            BOOST_CHECK_EQUAL(_DefaultCache.exists(oddKey), false);
            _DefaultCache.get(oddKey);
            BOOST_CHECK(false);
        } catch (std::exception& e) {

            BOOST_CHECK(true);
        }
    }

    _DefaultCache.setCapacity(50);  // update capacity to size 50
    // should pop items that are at the 'end' of the cache
    // thus, we'll check item key 98,96,...,2,0
    for (size_t k = 0; k < _DefaultCache.capacity() * 2; k += 2) {

        try {

            BOOST_CHECK_EQUAL(_DefaultCache.exists(k), false);
            _DefaultCache.get(k);
            BOOST_CHECK(false);
        } catch (std::exception& e) {
            BOOST_CHECK(true);
        }
    }

    BOOST_CHECK(
        _DefaultCache.exists(100));  // check that keys 100 and 198 still exist
    BOOST_CHECK(_DefaultCache.exists(198));
}

BOOST_FIXTURE_TEST_CASE(
    TryToInsertMorePairsThanCurrentCapacity,
    CreateCacheWithDefaultConstructorFixture)
{

    BOOST_CHECK_EQUAL(
        _DefaultCache.capacity(),
        200);  // cache should have initial capacity of 200
    BOOST_CHECK_EQUAL(_DefaultCache.size(), 0);  // size should be 0

    for (size_t key = 0; key <= _DefaultCache.capacity();
         key++) {  // fill item list 1 past capacity

        _DefaultCache.put(key, key + key);
    }

    BOOST_CHECK_EQUAL(
        _DefaultCache.capacity(),
        200);  // cache should still have capacity of 200
    BOOST_CHECK_EQUAL(_DefaultCache.size(), 200);  // size should be 200

    BOOST_CHECK_EQUAL(
        _DefaultCache.exists(0),
        false);  // item key 0 should be popped with the last
    // iteration of for loop above
}

BOOST_FIXTURE_TEST_CASE(
    TryToInsertIntoZeroCapacityCache, CreateCacheWithDefaultConstructorFixture)
{

    try {
        _DefaultCache.setCapacity(0);  // try to create negative cap cache
        _DefaultCache.put(0, 1);       // put shouldn't occur
        BOOST_CHECK(
            false);  // test should fail if allowed to create negative cap cache
    } catch (std::exception& e) {

        std::cout << e.what() << std::endl;
        BOOST_CHECK(true);  // handled correctly -- pass test
    }

    BOOST_CHECK_EQUAL(
        _DefaultCache.capacity(), 200);  // capacity should remain unchanged
    BOOST_CHECK_EQUAL(_DefaultCache.size(), 0);  // size should still be 0
}

BOOST_FIXTURE_TEST_CASE(PurgeTheCache, ReferenceBadKeyFixture)
{

    BOOST_CHECK_EQUAL(
        _Cache.capacity(), _Cache.size());  // capacity and size should be equal
    _Cache.purge();
    BOOST_CHECK_EQUAL(_Cache.size(), 0);  // size should still be 0
    BOOST_CHECK_EQUAL(
        _Cache.exists(1),
        false);  // item key 1 should not exist (implied by 0 size)
}
