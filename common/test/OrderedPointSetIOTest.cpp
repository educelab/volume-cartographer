#define BOOST_TEST_MODULE OrderedPointSetIOTest

#include <boost/test/unit_test.hpp>
#include "common/io/PointSetIO.h"
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

BOOST_FIXTURE_TEST_CASE(
    WriteThenReadBinaryOrderedPointSet, Point3iOrderedPointSet)
{
    // Binary IO is default
    PointSetIO<Point3i>::WriteOrderedPointSet("tmp.txt", ps);
    auto readPs = PointSetIO<Point3i>::OrderedPointSetFromFile("tmp.txt");
    BOOST_CHECK_EQUAL(readPs[0], ps(0, 0));
    BOOST_CHECK_EQUAL(readPs[1], ps(0, 1));
    BOOST_CHECK_EQUAL(readPs[2], ps(0, 2));
}

BOOST_FIXTURE_TEST_CASE(
    WriteThenReadAsciiOrderedPointSet, Point3iOrderedPointSet)
{
    PointSetIO<Point3i>::WriteOrderedPointSet("tmp.txt", ps, IOMode::ASCII);
    auto readPs =
        PointSetIO<Point3i>::OrderedPointSetFromFile("tmp.txt", IOMode::ASCII);
    BOOST_CHECK_EQUAL(readPs[0], ps(0, 0));
    BOOST_CHECK_EQUAL(readPs[1], ps(0, 1));
    BOOST_CHECK_EQUAL(readPs[2], ps(0, 2));
}
