#define BOOST_TEST_MODULE OrderedPointSetIOTest

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "common/io/PointSetIO.h"
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

BOOST_FIXTURE_TEST_CASE(
    WriteThenReadBinaryOrderedPointSet, Point3iOrderedPointSet)
{
    // Binary IO is default
    PointSetIO<Point3i>::WriteOrderedPointSet("binary.vcps", ps);
    auto readPs = PointSetIO<Point3i>::ReadOrderedPointSet("binary.vcps");
    BOOST_CHECK_EQUAL(readPs(0, 0), ps(0, 0));
    BOOST_CHECK_EQUAL(readPs(1, 0), ps(1, 0));
    BOOST_CHECK_EQUAL(readPs(2, 0), ps(2, 0));
}

BOOST_FIXTURE_TEST_CASE(
    WriteThenReadAsciiOrderedPointSet, Point3iOrderedPointSet)
{
    PointSetIO<Point3i>::WriteOrderedPointSet("ascii.vcps", ps, IOMode::ASCII);
    auto readPs =
        PointSetIO<Point3i>::ReadOrderedPointSet("ascii.vcps", IOMode::ASCII);
    BOOST_CHECK_EQUAL(readPs(0, 0), ps(0, 0));
    BOOST_CHECK_EQUAL(readPs(1, 0), ps(1, 0));
    BOOST_CHECK_EQUAL(readPs(2, 0), ps(2, 0));
}

BOOST_AUTO_TEST_CASE(FillOrderedPointSetStaticMethod)
{
    auto ps = OrderedPointSet<Point3i>::Fill(3, 1, {2, 1, 3});
    BOOST_CHECK_EQUAL(ps(0, 0), Point3i(2, 1, 3));
    BOOST_CHECK_EQUAL(ps(1, 0), Point3i(2, 1, 3));
    BOOST_CHECK_EQUAL(ps(2, 0), Point3i(2, 1, 3));
}
