#define BOOST_TEST_MODULE PointSetTest

#include "common/types/Point.h"
#include "common/types/PointSet.h"
#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace volcart;

struct EmptyPoint3iPointSet {
    EmptyPoint3iPointSet() : ps() {}

    PointSet<Point3i> ps;
};

struct Point3iPointSet {
    Point3iPointSet() : ps(3, 1)
    {
        ps.push_row({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    }

    PointSet<Point3i> ps;
};

BOOST_FIXTURE_TEST_CASE(CreateEmptyPointSet, EmptyPoint3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.size(), 0ul);
}

BOOST_FIXTURE_TEST_CASE(WriteThenReadPointSet, Point3iPointSet)
{
    std::cout << "write pointset" << std::endl;
    PointSet<Point3i>::writeFile("tmp.ps", ps);
    std::cout << "read pointset" << std::endl;
    auto readPs = PointSet<Point3i>::readFile("tmp.ps");
    BOOST_CHECK_EQUAL(readPs[0], ps[0]);
    BOOST_CHECK_EQUAL(readPs[1], ps[1]);
    BOOST_CHECK_EQUAL(readPs[2], ps[2]);
}
