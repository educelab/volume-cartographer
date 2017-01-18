#define BOOST_TEST_MODULE PointSetTest

#include <boost/test/unit_test.hpp>
#include <opencv2/core.hpp>

#include "core/types/PointSet.hpp"

using namespace volcart;

struct Vec3iPointSet {
    PointSet<cv::Vec3i> ps;

    Vec3iPointSet() : ps(3)
    {
        ps.push_back({1, 1, 1});
        ps.push_back({2, 2, 2});
        ps.push_back({3, 3, 3});
    }
};

BOOST_AUTO_TEST_CASE(EmptyPointSetTest)
{
    PointSet<cv::Vec3i> ps;
    BOOST_CHECK_EQUAL(ps.size(), 0);
    BOOST_CHECK(ps.empty());
}

BOOST_FIXTURE_TEST_CASE(OneRowPointSetTest, Vec3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.size(), 3);
    BOOST_CHECK(!ps.empty());
}

BOOST_FIXTURE_TEST_CASE(OneRowPointSetIteratorTest, Vec3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.front(), cv::Vec3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps.back(), cv::Vec3i(3, 3, 3));
    BOOST_CHECK_EQUAL(*std::begin(ps), cv::Vec3i(1, 1, 1));
    BOOST_CHECK_EQUAL(*(std::end(ps) - 1), cv::Vec3i(3, 3, 3));
    size_t i = 1;
    for (auto p : ps) {
        BOOST_CHECK_EQUAL(p, cv::Vec3i(i, i, i));
        ++i;
    }
}

BOOST_AUTO_TEST_CASE(FillPointSetTest)
{
    PointSet<cv::Vec3i> ps(3, {1, 1, 1});
    BOOST_CHECK_EQUAL(ps.size(), 3);
    for (auto p : ps) {
        BOOST_CHECK_EQUAL(p, cv::Vec3i(1, 1, 1));
    }
}

BOOST_FIXTURE_TEST_CASE(ClearPointSetTest, Vec3iPointSet)
{
    ps.clear();
    BOOST_CHECK_EQUAL(ps.size(), 0);
}

BOOST_FIXTURE_TEST_CASE(StatisticsPointSetTest, Vec3iPointSet)
{
    BOOST_CHECK_EQUAL(ps.min(), cv::Vec3i(1, 1, 1));
    BOOST_CHECK_EQUAL(ps.max(), cv::Vec3i(3, 3, 3));
    auto minmax = ps.minMax();
    BOOST_CHECK_EQUAL(minmax.first, cv::Vec3i(1, 1, 1));
    BOOST_CHECK_EQUAL(minmax.second, cv::Vec3i(3, 3, 3));
}

BOOST_AUTO_TEST_CASE(StatisticsEmptyPointSet)
{
    PointSet<cv::Vec3i> ps;
    BOOST_CHECK_THROW(ps.min(), std::range_error);
    BOOST_CHECK_THROW(ps.max(), std::range_error);
    BOOST_CHECK_THROW(ps.minMax(), std::range_error);
}

BOOST_FIXTURE_TEST_CASE(AppendPointSetToAnother, Vec3iPointSet)
{
    PointSet<cv::Vec3i> other(4);
    other.push_back({1, 2, 3});
    other.push_back({4, 5, 6});
    other.push_back({7, 8, 9});
    BOOST_CHECK_EQUAL(other.size(), 3);
    BOOST_CHECK_EQUAL(ps.size(), 3);

    // Append new pointset to old
    ps.append(other);

    BOOST_CHECK_EQUAL(ps.size(), 6);
    BOOST_CHECK_EQUAL(ps[3], cv::Vec3i(1, 2, 3));
    BOOST_CHECK_EQUAL(ps[4], cv::Vec3i(4, 5, 6));
    BOOST_CHECK_EQUAL(ps[5], cv::Vec3i(7, 8, 9));
}

BOOST_FIXTURE_TEST_CASE(AppendFullPointSetToFullPointSet, Vec3iPointSet)
{
    PointSet<cv::Vec3i> other(3);
    other.push_back({1, 2, 3});
    other.push_back({4, 5, 6});
    other.push_back({7, 8, 9});
    BOOST_CHECK_EQUAL(other.size(), 3);
    BOOST_CHECK_EQUAL(ps.size(), 3);

    // Append new pointset to old
    ps.append(other);

    BOOST_CHECK_EQUAL(ps.size(), 6);
    BOOST_CHECK_EQUAL(ps[3], cv::Vec3i(1, 2, 3));
    BOOST_CHECK_EQUAL(ps[4], cv::Vec3i(4, 5, 6));
    BOOST_CHECK_EQUAL(ps[5], cv::Vec3i(7, 8, 9));
}
