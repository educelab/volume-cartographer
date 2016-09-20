#define BOOST_TEST_MODULE OrderedPointSetTest

#include <boost/test/unit_test.hpp>
#include "common/types/Exceptions.h"
#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"

using namespace volcart;

struct Point3iOrderedPointSet {
    OrderedPointSet<Point3i> ps;

    Point3iOrderedPointSet() : ps(3, 1)
    {
        ps.push_row({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    }
};

BOOST_AUTO_TEST_CASE(ConstructEmptyOrderedPointSet)
{
    OrderedPointSet<Point3i> ps;
    BOOST_CHECK_EQUAL(ps.size(), 0);
    BOOST_CHECK_EQUAL(ps.width(), 0);
    BOOST_CHECK_EQUAL(ps.height(), 0);
}

BOOST_AUTO_TEST_CASE(PushRowAddsRow)
{
    OrderedPointSet<Point3i> ps{3, 1};
    std::vector<Point3i> points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    ps.push_row(points);
    BOOST_CHECK_EQUAL(ps(0, 0), Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps(1, 0), Point3i(2, 2, 2));
    BOOST_CHECK_EQUAL(ps(2, 0), Point3i(3, 3, 3));
}

BOOST_AUTO_TEST_CASE(FillOrderedPointSetStaticMethod)
{
    auto ps = OrderedPointSet<Point3i>::Fill(3, 1, {2, 1, 3});
    BOOST_CHECK_EQUAL(ps(0, 0), Point3i(2, 1, 3));
    BOOST_CHECK_EQUAL(ps(1, 0), Point3i(2, 1, 3));
    BOOST_CHECK_EQUAL(ps(2, 0), Point3i(2, 1, 3));
}

BOOST_FIXTURE_TEST_CASE(AppendOrderedPointSet, Point3iOrderedPointSet)
{
    auto other = OrderedPointSet<Point3i>::Fill(3, 4, {1, 3, 5});
    BOOST_CHECK_EQUAL(other.capacity(), 12);
    BOOST_CHECK_EQUAL(other.size(), 12);
    BOOST_CHECK_EQUAL(other.width(), 3);
    BOOST_CHECK_EQUAL(other.height(), 4);
    BOOST_CHECK_EQUAL(ps.capacity(), 3);
    BOOST_CHECK_EQUAL(ps.size(), 3);
    BOOST_CHECK_EQUAL(ps.width(), 3);
    BOOST_CHECK_EQUAL(ps.height(), 1);

    // Append other to ps
    ps.append(other);

    BOOST_CHECK_EQUAL(ps.width(), 3);
    BOOST_CHECK_EQUAL(ps.height(), 5);
    BOOST_CHECK_EQUAL(ps.size(), 15);
    BOOST_CHECK_EQUAL(ps.capacity(), 15);
    BOOST_CHECK_EQUAL(ps(0, 1), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 1), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 1), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 2), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 2), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 2), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 3), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 3), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 3), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 4), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 4), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 4), Point3i(1, 3, 5));
}

BOOST_FIXTURE_TEST_CASE(AppendNonFullOrderedPointSet, Point3iOrderedPointSet)
{
    OrderedPointSet<Point3i> other(3, 4);
    other.push_row({{1, 1, 1}, {1, 2, 3}, {1, 3, 5}});
    other.push_row({{2, 2, 2}, {2, 2, 3}, {2, 3, 5}});
    other.push_row({{2, 2, 2}, {2, 2, 3}, {2, 3, 5}});

    BOOST_CHECK_EQUAL(other.size(), 9);
    BOOST_CHECK_EQUAL(other.capacity(), 12);
    BOOST_CHECK_EQUAL(other.width(), 3);
    BOOST_CHECK_EQUAL(other.height(), 4);

    ps.append(other);

    BOOST_CHECK_EQUAL(ps.size(), 12);
    BOOST_CHECK_EQUAL(ps.capacity(), 15);
    BOOST_CHECK_EQUAL(ps.width(), 3);
    BOOST_CHECK_EQUAL(ps.height(), 5);
    BOOST_CHECK_EQUAL(ps(0, 1), Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps(1, 1), Point3i(1, 2, 3));
    BOOST_CHECK_EQUAL(ps(2, 1), Point3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 2), Point3i(2, 2, 2));
    BOOST_CHECK_EQUAL(ps(1, 2), Point3i(2, 2, 3));
    BOOST_CHECK_EQUAL(ps(2, 2), Point3i(2, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 3), Point3i(2, 2, 2));
    BOOST_CHECK_EQUAL(ps(1, 3), Point3i(2, 2, 3));
    BOOST_CHECK_EQUAL(ps(2, 3), Point3i(2, 3, 5));
}
