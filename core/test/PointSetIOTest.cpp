#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>

#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/PointSet.hpp"

constexpr auto TEST_HEADER_FILENAME = "test_header.txt";

using namespace volcart;

void writeTestHeader(const std::string& testHeader);

class Point3iUnorderedPointSet : public ::testing::Test
{
public:
    PointSet<cv::Vec3i> ps;

    Point3iUnorderedPointSet() : ps(3)
    {
        ps.push_back({1, 1, 1});
        ps.push_back({2, 2, 2});
        ps.push_back({3, 3, 3});
    }
};

TEST_F(Point3iUnorderedPointSet, WriteThenReadBinaryUnorderedPointSet)
{
    // Binary IO is default
    PointSetIO<cv::Vec3i>::WritePointSet("tmp.txt", ps);
    auto readPs = PointSetIO<cv::Vec3i>::ReadPointSet("tmp.txt");
    EXPECT_EQ(readPs[0], ps[0]);
    EXPECT_EQ(readPs[1], ps[1]);
    EXPECT_EQ(readPs[2], ps[2]);
}

TEST_F(Point3iUnorderedPointSet, WriteThenReadAsciiUnorderedPointSet)
{
    PointSetIO<cv::Vec3i>::WritePointSet("tmp.txt", ps, IOMode::ASCII);
    auto readPs = PointSetIO<cv::Vec3i>::ReadPointSet("tmp.txt", IOMode::ASCII);
    EXPECT_EQ(readPs[0], ps[0]);
    EXPECT_EQ(readPs[1], ps[1]);
    EXPECT_EQ(readPs[2], ps[2]);
}

// Utility method for writing a test header defined in a test case
void writeTestHeader(const std::string& testHeader)
{
    std::ofstream out{TEST_HEADER_FILENAME};
    out << testHeader;
}

TEST(PointSetIOTest, ParseHeaderIgnoreComments)
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
    auto header = PointSetIO<cv::Vec3i>::ParseHeader(inHeader, true);

    EXPECT_EQ(header.width, 3);
    EXPECT_EQ(header.height, 1);
    EXPECT_EQ(header.ordered, true);
    EXPECT_EQ(header.type, "int");
    EXPECT_EQ(header.dim, 3);
}

TEST(PointSetIOTest, ParseHeaderOffsetKeywords)
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
    auto header = PointSetIO<cv::Vec3i>::ParseHeader(inHeader, true);

    EXPECT_EQ(header.width, 3);
    EXPECT_EQ(header.height, 1);
    EXPECT_EQ(header.ordered, true);
    EXPECT_EQ(header.type, "int");
    EXPECT_EQ(header.dim, 3);
}

TEST(PointSetIOTest, UnorderedPointSetWithWidthThrows)
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

    EXPECT_THROW(
        PointSetIO<cv::Vec3i>::ParseHeader(inHeader, false), IOException);
}

TEST(PointSetIOTest, OrderedPointSetWithSizeThrows)
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

    EXPECT_THROW(
        PointSetIO<cv::Vec3i>::ParseHeader(inHeader, true), IOException);
}

TEST(PointSetIOTest, PointSetWithoutDimThrows)
{
    const std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "type: int\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    EXPECT_THROW(
        PointSetIO<cv::Vec3i>::ParseHeader(inHeader, false), IOException);
}

TEST(PointSetIOTest, PointSetWithoutTypeThrows)
{
    const std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "dim: 3\n"
        "version: 1\n"
        "<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    EXPECT_THROW(
        PointSetIO<cv::Vec3i>::ParseHeader(inHeader, false), IOException);
}

TEST(PointSetIOTest, PointSetWithWrongTypeThrows)
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

    EXPECT_THROW(
        PointSetIO<cv::Vec3i>::ParseHeader(inHeader, false), IOException);
}

TEST(PointSetIOTest, PointSetWithWrongVersionThrows)
{
    std::string commentHeader =
        "size: 3\n"
        "ordered: false\n"
        "dim: 3\n"
        "type: int\n";

    commentHeader +=
        "version: " + std::to_string(PointSet<cv::Vec3i>::FORMAT_VERSION + 1) +
        "\n<>\n";

    writeTestHeader(commentHeader);
    std::ifstream inHeader(TEST_HEADER_FILENAME);

    EXPECT_THROW(
        PointSetIO<cv::Vec3i>::ParseHeader(inHeader, false), IOException);
}

TEST(PointSetIOTest, PointSetWithWrongDimThrows)
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

    EXPECT_THROW(
        PointSetIO<cv::Vec3i>::ParseHeader(inHeader, false), IOException);
}