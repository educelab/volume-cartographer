#include <gtest/gtest.h>

#include <iostream>

#include <opencv2/core.hpp>

#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/OrderedPointSet.hpp"

using namespace volcart;

class OrderedPointSetIO : public ::testing::Test
{
public:
    OrderedPointSet<cv::Vec3i> ps;
    std::string path{"vc_core_OrderedPointSetIO_"};

    OrderedPointSetIO() : ps{3}
    {
        ps.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    }
};

TEST_F(OrderedPointSetIO, WriteReadBinary)
{
    // Write to disk
    // Binary IO is default
    path += "WriteReadBinary.vcps";
    EXPECT_NO_THROW(PointSetIO<cv::Vec3i>::WriteOrderedPointSet(path, ps));

    // Read from disk
    OrderedPointSet<cv::Vec3i> read;
    EXPECT_NO_THROW(read = PointSetIO<cv::Vec3i>::ReadOrderedPointSet(path));

    // Check values
    EXPECT_EQ(read(0, 0), ps(0, 0));
    EXPECT_EQ(read(0, 1), ps(0, 1));
    EXPECT_EQ(read(0, 2), ps(0, 2));
}

TEST_F(OrderedPointSetIO, WriteReadASCII)
{
    // Write to disk
    path += "WriteReadASCII.vcps";
    EXPECT_NO_THROW(
        PointSetIO<cv::Vec3i>::WriteOrderedPointSet(path, ps, IOMode::ASCII));

    // Read from disk
    OrderedPointSet<cv::Vec3i> read;
    EXPECT_NO_THROW(
        read = PointSetIO<cv::Vec3i>::ReadOrderedPointSet(path, IOMode::ASCII));

    // Check values
    EXPECT_EQ(read(0, 0), ps(0, 0));
    EXPECT_EQ(read(0, 1), ps(0, 1));
    EXPECT_EQ(read(0, 2), ps(0, 2));
}