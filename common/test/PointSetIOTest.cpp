#define BOOST_TEST_MODULE PointSetIOTest

#include <fstream>
#include <iostream>
#include <string>
#include <boost/test/unit_test.hpp>
#include "common/io/PointSetIO.h"
#include "common/types/Point.h"
#include "common/types/PointSet.h"

constexpr auto TEST_HEADER_FILENAME = "test_header.txt";

using namespace volcart;

struct Point3iUnorderedPointSet {
    PointSet<Point3i> ps;

    Point3iUnorderedPointSet() : ps(3)
    {
        ps.push_back({1, 1, 1});
        ps.push_back({2, 2, 2});
        ps.push_back({3, 3, 3});
    }
};

BOOST_FIXTURE_TEST_CASE(
    WriteThenReadBinaryUnorderedPointSet, Point3iUnorderedPointSet)
{
    // Binary IO is default
    PointSetIO<Point3i>::WritePointSet("tmp.txt", ps);
    auto readPs = PointSetIO<Point3i>::PointSetFromFile("tmp.txt");
    BOOST_CHECK_EQUAL(readPs[0], ps[0]);
    BOOST_CHECK_EQUAL(readPs[1], ps[1]);
    BOOST_CHECK_EQUAL(readPs[2], ps[2]);
}

BOOST_FIXTURE_TEST_CASE(
    WriteThenReadAsciiUnorderedPointSet, Point3iUnorderedPointSet)
{
    PointSetIO<Point3i>::WritePointSet("tmp.txt", ps, IOMode::ASCII);
    auto readPs =
        PointSetIO<Point3i>::PointSetFromFile("tmp.txt", IOMode::ASCII);
    BOOST_CHECK_EQUAL(readPs[0], ps[0]);
    BOOST_CHECK_EQUAL(readPs[1], ps[1]);
    BOOST_CHECK_EQUAL(readPs[2], ps[2]);
}

// Utility method for writing a test header defined in a test case
void writeTestHeader(const std::string& testHeader)
{
    std::ofstream out{TEST_HEADER_FILENAME};
    out << testHeader;
}

BOOST_AUTO_TEST_CASE(ParseHeaderIgnoreComments)
{
    const std::string commentHeader =
        "# This is a comment. It doesn't affect the header parser\n"
        "      # This is an offset comment. It also doesn't affect it\n"
        "#This is a comment without a leading # and <space>\n"
        "# The next line starts the header info\n"
        "width: 3\n"
        "height: 1\n"
        "ordered: true\n"
        "type: int\n"
        "dim: 3\n"
        "version: 1\n"
        "# This is a comment inside the header info. Next line is the end\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);
    auto header = PointSetIO<Point3i>::ParseHeader(inHeader, true);

    BOOST_CHECK_EQUAL(header.width, 3);
    BOOST_CHECK_EQUAL(header.height, 1);
    BOOST_CHECK_EQUAL(header.ordered, true);
    BOOST_CHECK_EQUAL(header.type, "int");
    BOOST_CHECK_EQUAL(header.dim, 3);
}

BOOST_AUTO_TEST_CASE(ParseHeaderOffsetKeywords)
{
    const std::string commentHeader =
        "       width: 3\n"
        " height: 1     \n"
        "               ordered: true\n"
        " type: int\n"
        "    dim: 3   \n"
        "version: 1        \n"
        " <>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);
    auto header = PointSetIO<Point3i>::ParseHeader(inHeader, true);

    BOOST_CHECK_EQUAL(header.width, 3);
    BOOST_CHECK_EQUAL(header.height, 1);
    BOOST_CHECK_EQUAL(header.ordered, true);
    BOOST_CHECK_EQUAL(header.type, "int");
    BOOST_CHECK_EQUAL(header.dim, 3);
}

BOOST_AUTO_TEST_CASE(UnorderedPointSetWithWidthThrows)
{
    const std::string commentHeader =
        "width: 3\n"
        "ordered: false\n"
        "type: int\n"
        "dim: 3\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    BOOST_CHECK_THROW(
        PointSetIO<Point3i>::ParseHeader(inHeader, false), IOException);
}

BOOST_AUTO_TEST_CASE(OrderedPointSetWithSizeThrows)
{
    const std::string commentHeader =
        "size: 3\n"
        "ordered: true\n"
        "type: int\n"
        "dim: 3\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    BOOST_CHECK_THROW(
        PointSetIO<Point3i>::ParseHeader(inHeader, true), IOException);
}

BOOST_AUTO_TEST_CASE(PointSetWithoutDimThrows)
{
    const std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "type: int\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    BOOST_CHECK_THROW(
        PointSetIO<Point3i>::ParseHeader(inHeader, false), IOException);
}

BOOST_AUTO_TEST_CASE(PointSetWithoutTypeThrows)
{
    const std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "dim: 3\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    BOOST_CHECK_THROW(
        PointSetIO<Point3i>::ParseHeader(inHeader, false), IOException);
}

BOOST_AUTO_TEST_CASE(PointSetWithWrongTypeThrows)
{
    const std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "dim: 3\n"
        "type: double\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    BOOST_CHECK_THROW(
        PointSetIO<Point3i>::ParseHeader(inHeader, false), IOException);
}

BOOST_AUTO_TEST_CASE(PointSetWithWrongVersionThrows)
{
    std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "dim: 3\n"
        "type: int\n";

    commentHeader += "version: " +
                     std::to_string(PointSet<Point3i>::FORMAT_VERSION + 1) +
                     "\n<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    BOOST_CHECK_THROW(
        PointSetIO<Point3i>::ParseHeader(inHeader, false), IOException);
}

BOOST_AUTO_TEST_CASE(PointSetWithWrongDimThrows)
{
    const std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "dim: 4\n"
        "type: int\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    BOOST_CHECK_THROW(
        PointSetIO<Point3i>::ParseHeader(inHeader, false), IOException);
}
