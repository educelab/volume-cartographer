#define BOOST_TEST_MODULE PointSetTest

#include <boost/test/unit_test.hpp>
#include "common/types/Point.h"
#include "common/types/PointSet.h"

using namespace volcart;

struct Point3iPointSet {
    PointSet<Point3i> ps;

    Point3iPointSet() : ps(3)
    {
        ps.push_back({1, 1, 1});
        ps.push_back({2, 2, 2});
        ps.push_back({3, 3, 3});
    }
};

BOOST_AUTO_TEST_CASE(EmptyPointSetTest)
{
    PointSet<Point3i> ps;
    BOOST_CHECK_EQUAL(ps.size(), 0);
    BOOST_CHECK(ps.empty());
}

BOOST_FIXTURE_TEST_CASE(OneRowPointSetTest, Point3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.size(), 3);
    BOOST_CHECK(!ps.empty());
}

BOOST_FIXTURE_TEST_CASE(OneRowPointSetIteratorTest, Point3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.front(), Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps.back(), Point3i(3, 3, 3));
    BOOST_CHECK_EQUAL(*std::begin(ps), Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(*(std::end(ps) - 1), Point3i(3, 3, 3));
    size_t i = 1;
    for (auto p : ps) {
        Point3i tmp(i, i, i);
        BOOST_CHECK_EQUAL(p, tmp);
        ++i;
    }
}

BOOST_AUTO_TEST_CASE(FillPointSetTest)
{
    PointSet<Point3i> ps(3, {1, 1, 1});
    BOOST_CHECK_EQUAL(ps.size(), 3);
    for (auto p : ps) {
        BOOST_CHECK_EQUAL(p, Point3i(1, 1, 1));
    }
}

BOOST_FIXTURE_TEST_CASE(StatisticsPointSetTest, Point3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.min(), Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps.max(), Point3i(3, 3, 3));
    auto minmax = ps.min_max();
    BOOST_CHECK_EQUAL(minmax.first, Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(minmax.second, Point3i(3, 3, 3));
}

BOOST_AUTO_TEST_CASE(StatisticsEmptyPointSet)
{
    PointSet<Point3i> ps;
    BOOST_CHECK_THROW(ps.min(), std::range_error);
    BOOST_CHECK_THROW(ps.max(), std::range_error);
    BOOST_CHECK_THROW(ps.min_max(), std::range_error);
}
