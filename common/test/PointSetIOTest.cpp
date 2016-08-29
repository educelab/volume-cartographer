#define BOOST_TEST_MODULE PointSetIOTest

#include "common/types/Point.h"
#include "common/types/PointSet.h"
#include "common/types/PointSetIO.h"
#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace volcart;

struct Point3iOrderedPointSet {
    PointSet<Point3i> ps;

    Point3iOrderedPointSet() : ps(3, 1)
    {
        ps.push_row({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    }
};

struct Point3iUnorderedPointSet {
    PointSet<Point3i> ps;

    Point3iUnorderedPointSet() : ps(3)
    {
        ps.push_back({1, 1, 1});
        ps.push_back({2, 2, 2});
        ps.push_back({3, 3, 3});
    }
};

BOOST_FIXTURE_TEST_CASE(WriteThenReadBinaryOrderedPointSet,
                        Point3iOrderedPointSet)
{
    // Binary IO is default
    PointSetIO<Point3i>::writeFile("tmp.txt", ps);
    auto readPs = PointSetIO<Point3i>::readFile("tmp.txt");
    BOOST_CHECK_EQUAL(readPs[0], ps[0]);
    BOOST_CHECK_EQUAL(readPs[1], ps[1]);
    BOOST_CHECK_EQUAL(readPs[2], ps[2]);
}

BOOST_FIXTURE_TEST_CASE(WriteThenReadAsciiOrderedPointSet,
                        Point3iOrderedPointSet)
{
    PointSetIO<Point3i>::writeFile("tmp.txt", ps, IOMode::ASCII);
    auto readPs = PointSetIO<Point3i>::readFile("tmp.txt", IOMode::ASCII);
    BOOST_CHECK_EQUAL(readPs[0], ps[0]);
    BOOST_CHECK_EQUAL(readPs[1], ps[1]);
    BOOST_CHECK_EQUAL(readPs[2], ps[2]);
}

BOOST_FIXTURE_TEST_CASE(WriteThenReadBinaryUnorderedPointSet,
                        Point3iUnorderedPointSet)
{
    // Binary IO is default
    PointSetIO<Point3i>::writeFile("tmp.txt", ps);
    auto readPs = PointSetIO<Point3i>::readFile("tmp.txt");
    BOOST_CHECK_EQUAL(readPs[0], ps[0]);
    BOOST_CHECK_EQUAL(readPs[1], ps[1]);
    BOOST_CHECK_EQUAL(readPs[2], ps[2]);
}

BOOST_FIXTURE_TEST_CASE(WriteThenReadAsciiUnorderedPointSet,
                        Point3iUnorderedPointSet)
{
    PointSetIO<Point3i>::writeFile("tmp.txt", ps, IOMode::ASCII);
    auto readPs = PointSetIO<Point3i>::readFile("tmp.txt", IOMode::ASCII);
    BOOST_CHECK_EQUAL(readPs[0], ps[0]);
    BOOST_CHECK_EQUAL(readPs[1], ps[1]);
    BOOST_CHECK_EQUAL(readPs[2], ps[2]);
}
