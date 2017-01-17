#define BOOST_TEST_MODULE OrderedPointSetTest

#include <boost/test/unit_test.hpp>
#include <opencv2/core.hpp>

#include "core/types/Exceptions.hpp"
#include "core/types/OrderedPointSet.hpp"

using namespace volcart;

struct Vec3iOrderedPointSet {
    OrderedPointSet<cv::Vec3i> ps;

    Vec3iOrderedPointSet() : ps(3)
    {
        ps.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    }
};

struct Vec3iOrderedPointSet4Rows {
    OrderedPointSet<cv::Vec3i> ps;

    Vec3iOrderedPointSet4Rows() : ps(3)
    {
        ps.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
        ps.pushRow({{4, 4, 4}, {4, 4, 4}, {4, 4, 4}});
        ps.pushRow({{2, 2, 2}, {2, 2, 2}, {2, 2, 2}});
        ps.pushRow({{1, 1, 1}, {1, 1, 1}, {1, 1, 1}});
    }
};

BOOST_AUTO_TEST_CASE(ConstructEmptyOrderedPointSet)
{
    OrderedPointSet<cv::Vec3i> ps;
    BOOST_CHECK_EQUAL(ps.size(), 0);
    BOOST_CHECK_EQUAL(ps.width(), 0);
    BOOST_CHECK_EQUAL(ps.height(), 0);
}

BOOST_AUTO_TEST_CASE(SetWidth)
{
    OrderedPointSet<cv::Vec3i> ps;
    ps.setWidth(3);
    BOOST_CHECK_EQUAL(ps.width(), 3);
    std::vector<cv::Vec3i> points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    BOOST_CHECK_NO_THROW(ps.pushRow(points));
    BOOST_CHECK_THROW(ps.setWidth(4), std::logic_error);
}

BOOST_FIXTURE_TEST_CASE(
    ResetPointSetClearsAndSetsWidthToZero, Vec3iOrderedPointSet)
{
    ps.reset();
    BOOST_CHECK_EQUAL(ps.width(), 0);
    BOOST_CHECK_EQUAL(ps.size(), 0);
    BOOST_CHECK_NO_THROW(ps.setWidth(3));
}

BOOST_AUTO_TEST_CASE(PushRowAddsRow)
{
    OrderedPointSet<cv::Vec3i> ps{3};
    std::vector<cv::Vec3i> points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    ps.pushRow(points);
    BOOST_CHECK_EQUAL(ps(0, 0), cv::Vec3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps(1, 0), cv::Vec3i(2, 2, 2));
    BOOST_CHECK_EQUAL(ps(2, 0), cv::Vec3i(3, 3, 3));
}

BOOST_AUTO_TEST_CASE(FillOrderedPointSetStaticMethod)
{
    auto ps = OrderedPointSet<cv::Vec3i>::Fill(3, 1, {2, 1, 3});
    BOOST_CHECK_EQUAL(ps(0, 0), cv::Vec3i(2, 1, 3));
    BOOST_CHECK_EQUAL(ps(1, 0), cv::Vec3i(2, 1, 3));
    BOOST_CHECK_EQUAL(ps(2, 0), cv::Vec3i(2, 1, 3));
}

BOOST_FIXTURE_TEST_CASE(GetRowFromPointSet, Vec3iOrderedPointSet)
{
    auto row = ps.getRow(0);
    BOOST_CHECK_EQUAL(row[0], cv::Vec3i(1, 1, 1));
    BOOST_CHECK_EQUAL(row[1], cv::Vec3i(2, 2, 2));
    BOOST_CHECK_EQUAL(row[2], cv::Vec3i(3, 3, 3));
}

BOOST_FIXTURE_TEST_CASE(WrongRowInGetRowThrows, Vec3iOrderedPointSet)
{
    BOOST_CHECK_THROW(ps.getRow(2), std::range_error);
}

BOOST_FIXTURE_TEST_CASE(CopyRowsFromOrderedPointSet, Vec3iOrderedPointSet4Rows)
{
    auto newPs = ps.copyRows(0, 2);
    BOOST_CHECK_EQUAL(newPs.width(), 3);
    BOOST_CHECK_EQUAL(newPs.height(), 3);
    BOOST_CHECK_EQUAL(newPs.size(), 9);
    BOOST_CHECK_EQUAL(newPs(0, 0), cv::Vec3i(1, 1, 1));
    BOOST_CHECK_EQUAL(newPs(1, 0), cv::Vec3i(2, 2, 2));
    BOOST_CHECK_EQUAL(newPs(2, 0), cv::Vec3i(3, 3, 3));
    BOOST_CHECK_EQUAL(newPs(0, 1), cv::Vec3i(4, 4, 4));
    BOOST_CHECK_EQUAL(newPs(1, 1), cv::Vec3i(4, 4, 4));
    BOOST_CHECK_EQUAL(newPs(2, 1), cv::Vec3i(4, 4, 4));
    BOOST_CHECK_EQUAL(newPs(0, 2), cv::Vec3i(2, 2, 2));
    BOOST_CHECK_EQUAL(newPs(1, 2), cv::Vec3i(2, 2, 2));
    BOOST_CHECK_EQUAL(newPs(2, 2), cv::Vec3i(2, 2, 2));
}

BOOST_FIXTURE_TEST_CASE(
    CopyRowsWithGreaterFirstIndexThrows, Vec3iOrderedPointSet4Rows)
{
    BOOST_CHECK_THROW(ps.copyRows(3, 2), std::logic_error);
}

BOOST_FIXTURE_TEST_CASE(
    CopyRowsWithOutOfRangeIndexThrows, Vec3iOrderedPointSet4Rows)
{
    BOOST_CHECK_THROW(ps.copyRows(0, 5), std::range_error);
}

BOOST_FIXTURE_TEST_CASE(AppendOrderedPointSet, Vec3iOrderedPointSet)
{
    auto other = OrderedPointSet<cv::Vec3i>::Fill(3, 4, {1, 3, 5});
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
    BOOST_CHECK_EQUAL(ps(0, 1), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 1), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 1), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 2), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 2), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 2), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 3), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 3), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 3), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(0, 4), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(1, 4), cv::Vec3i(1, 3, 5));
    BOOST_CHECK_EQUAL(ps(2, 4), cv::Vec3i(1, 3, 5));
}

BOOST_FIXTURE_TEST_CASE(AppendWiderPointSetThrows, Vec3iOrderedPointSet)
{
    OrderedPointSet<cv::Vec3i> other{4};
    other.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}, {4, 4, 4}});
    BOOST_CHECK_THROW(ps.append(other), std::logic_error);
}
