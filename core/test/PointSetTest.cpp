#define BOOST_TEST_MODULE PointSetTest

#include <boost/test/unit_test.hpp>
#include "core/types/Point.h"
#include "core/types/PointSet.h"

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
        BOOST_CHECK_EQUAL(p, Point3i(i, i, i));
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

BOOST_FIXTURE_TEST_CASE(ClearPointSetTest, Point3iPointSet)
{
    ps.clear();
    BOOST_CHECK_EQUAL(ps.size(), 0);
}

BOOST_FIXTURE_TEST_CASE(StatisticsPointSetTest, Point3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.min(), Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps.max(), Point3i(3, 3, 3));
    auto minmax = ps.minMax();
    BOOST_CHECK_EQUAL(minmax.first, Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(minmax.second, Point3i(3, 3, 3));
}

BOOST_AUTO_TEST_CASE(StatisticsEmptyPointSet)
{
    PointSet<Point3i> ps;
    BOOST_CHECK_THROW(ps.min(), std::range_error);
    BOOST_CHECK_THROW(ps.max(), std::range_error);
    BOOST_CHECK_THROW(ps.minMax(), std::range_error);
}

BOOST_FIXTURE_TEST_CASE(AppendPointSetToAnother, Point3iPointSet)
{
    PointSet<Point3i> other(4);
    other.push_back({1, 2, 3});
    other.push_back({4, 5, 6});
    other.push_back({7, 8, 9});
    BOOST_CHECK_EQUAL(other.size(), 3);
    BOOST_CHECK_EQUAL(ps.size(), 3);

    // Append new pointset to old
    ps.append(other);

    BOOST_CHECK_EQUAL(ps.size(), 6);
    BOOST_CHECK_EQUAL(ps[3], Point3i(1, 2, 3));
    BOOST_CHECK_EQUAL(ps[4], Point3i(4, 5, 6));
    BOOST_CHECK_EQUAL(ps[5], Point3i(7, 8, 9));
}

BOOST_FIXTURE_TEST_CASE(AppendFullPointSetToFullPointSet, Point3iPointSet)
{
    PointSet<Point3i> other(3);
    other.push_back({1, 2, 3});
    other.push_back({4, 5, 6});
    other.push_back({7, 8, 9});
    BOOST_CHECK_EQUAL(other.size(), 3);
    BOOST_CHECK_EQUAL(ps.size(), 3);

    // Append new pointset to old
    ps.append(other);

    BOOST_CHECK_EQUAL(ps.size(), 6);
    BOOST_CHECK_EQUAL(ps[3], Point3i(1, 2, 3));
    BOOST_CHECK_EQUAL(ps[4], Point3i(4, 5, 6));
    BOOST_CHECK_EQUAL(ps[5], Point3i(7, 8, 9));
}
