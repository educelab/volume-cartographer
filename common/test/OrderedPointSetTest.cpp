#define BOOST_TEST_MODULE OrderedPointSetTest

#include <boost/test/unit_test.hpp>
#include "common/types/Exceptions.h"
#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"

using namespace volcart;

struct Point3iOrderedPointSet {
    OrderedPointSet<Point3i> ps;

    Point3iOrderedPointSet() : ps(3)
    {
        ps.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    }
};

struct Point3iOrderedPointSet4Rows {
    OrderedPointSet<Point3i> ps;

    Point3iOrderedPointSet4Rows() : ps(3)
    {
        ps.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
        ps.pushRow({{4, 4, 4}, {4, 4, 4}, {4, 4, 4}});
        ps.pushRow({{2, 2, 2}, {2, 2, 2}, {2, 2, 2}});
        ps.pushRow({{1, 1, 1}, {1, 1, 1}, {1, 1, 1}});
    }
};

BOOST_AUTO_TEST_CASE(ConstructEmptyOrderedPointSet)
{
    OrderedPointSet<Point3i> ps;
    BOOST_CHECK_EQUAL(ps.size(), 0);
    BOOST_CHECK_EQUAL(ps.width(), 0);
    BOOST_CHECK_EQUAL(ps.height(), 0);
}

BOOST_AUTO_TEST_CASE(SetWidth)
{
    OrderedPointSet<Point3i> ps;
    ps.setWidth(3);
    BOOST_CHECK_EQUAL(ps.width(), 3);
    std::vector<volcart::Point3i> points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    BOOST_CHECK_NO_THROW(ps.pushRow(points));
    BOOST_CHECK_THROW(ps.setWidth(4), std::logic_error);
}

BOOST_FIXTURE_TEST_CASE(
    ResetPointSetClearsAndSetsWidthToZero, Point3iOrderedPointSet)
{
    ps.reset();
    BOOST_CHECK_EQUAL(ps.width(), 0);
    BOOST_CHECK_EQUAL(ps.size(), 0);
    BOOST_CHECK_NO_THROW(ps.setWidth(3));
}

BOOST_AUTO_TEST_CASE(PushRowAddsRow)
{
    OrderedPointSet<Point3i> ps{3};
    std::vector<Point3i> points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    ps.pushRow(points);
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

BOOST_FIXTURE_TEST_CASE(GetRowFromPointSet, Point3iOrderedPointSet)
{
    auto row = ps.getRow(0);
    BOOST_CHECK_EQUAL(row[0], Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(row[1], Point3i(2, 2, 2));
    BOOST_CHECK_EQUAL(row[2], Point3i(3, 3, 3));
}

BOOST_FIXTURE_TEST_CASE(WrongRowInGetRowThrows, Point3iOrderedPointSet)
{
    BOOST_CHECK_THROW(ps.getRow(2), std::range_error);
}

BOOST_FIXTURE_TEST_CASE(
    CopyRowsFromOrderedPointSet, Point3iOrderedPointSet4Rows)
{
    auto newPs = ps.copyRows(0, 2);
    BOOST_CHECK_EQUAL(newPs.width(), 3);
    BOOST_CHECK_EQUAL(newPs.height(), 3);
    BOOST_CHECK_EQUAL(newPs.size(), 9);
    BOOST_CHECK_EQUAL(newPs(0, 0), Point3i(1, 1, 1));
    BOOST_CHECK_EQUAL(newPs(1, 0), Point3i(2, 2, 2));
    BOOST_CHECK_EQUAL(newPs(2, 0), Point3i(3, 3, 3));
    BOOST_CHECK_EQUAL(newPs(0, 1), Point3i(4, 4, 4));
    BOOST_CHECK_EQUAL(newPs(1, 1), Point3i(4, 4, 4));
    BOOST_CHECK_EQUAL(newPs(2, 1), Point3i(4, 4, 4));
    BOOST_CHECK_EQUAL(newPs(0, 2), Point3i(2, 2, 2));
    BOOST_CHECK_EQUAL(newPs(1, 2), Point3i(2, 2, 2));
    BOOST_CHECK_EQUAL(newPs(2, 2), Point3i(2, 2, 2));
}

BOOST_FIXTURE_TEST_CASE(
    CopyRowsWithGreaterFirstIndexThrows, Point3iOrderedPointSet4Rows)
{
    BOOST_CHECK_THROW(ps.copyRows(3, 2), std::logic_error);
}

BOOST_FIXTURE_TEST_CASE(
    CopyRowsWithOutOfRangeIndexThrows, Point3iOrderedPointSet4Rows)
{
    BOOST_CHECK_THROW(ps.copyRows(0, 5), std::range_error);
}

BOOST_FIXTURE_TEST_CASE(AppendOrderedPointSet, Point3iOrderedPointSet)
{
    auto other = OrderedPointSet<Point3i>::Fill(3, 4, {1, 3, 5});
    BOOST_CHECK_EQUAL(other.size(), 12);
    BOOST_CHECK_EQUAL(other.width(), 3);
    BOOST_CHECK_EQUAL(other.height(), 4);
    BOOST_CHECK_EQUAL(ps.size(), 3);
    BOOST_CHECK_EQUAL(ps.width(), 3);
    BOOST_CHECK_EQUAL(ps.height(), 1);

    // Append other to ps
    ps.append(other);

    BOOST_CHECK_EQUAL(ps.width(), 3);
    BOOST_CHECK_EQUAL(ps.height(), 5);
    BOOST_CHECK_EQUAL(ps.size(), 15);
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

BOOST_FIXTURE_TEST_CASE(AppendWiderPointSetThrows, Point3iOrderedPointSet)
{
    OrderedPointSet<Point3i> other{4};
    other.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}, {4, 4, 4}});
    BOOST_CHECK_THROW(ps.append(other), std::logic_error);
}
