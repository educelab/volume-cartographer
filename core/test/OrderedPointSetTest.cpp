#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "vc/core/types/OrderedPointSet.hpp"

namespace vc = volcart;

class OrderedPointSet : public ::testing::Test
{
public:
    vc::OrderedPointSet<cv::Vec3i> ps;

    OrderedPointSet() : ps(3)
    {
        ps.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
        ps.pushRow({{4, 4, 4}, {4, 4, 4}, {4, 4, 4}});
        ps.pushRow({{2, 2, 2}, {2, 2, 2}, {2, 2, 2}});
        ps.pushRow({{1, 1, 1}, {1, 1, 1}, {1, 1, 1}});
    }
};

TEST_F(OrderedPointSet, EmptyConstructor)
{
    ps = vc::OrderedPointSet<cv::Vec3i>();
    EXPECT_EQ(ps.size(), 0);
    EXPECT_EQ(ps.width(), 0);
    EXPECT_EQ(ps.height(), 0);
}

TEST_F(OrderedPointSet, SetWidth)
{
    ps = vc::OrderedPointSet<cv::Vec3i>();
    ps.setWidth(3);
    EXPECT_EQ(ps.width(), 3);
    std::vector<cv::Vec3i> points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    EXPECT_NO_THROW(ps.pushRow(points));
    EXPECT_THROW(ps.setWidth(4), std::logic_error);
}

TEST_F(OrderedPointSet, Reset)
{
    ps.reset();
    EXPECT_EQ(ps.width(), 0);
    EXPECT_EQ(ps.size(), 0);
    EXPECT_NO_THROW(ps.setWidth(3));
}

TEST_F(OrderedPointSet, PushRow)
{
    ps = vc::OrderedPointSet<cv::Vec3i>{3};
    std::vector<cv::Vec3i> points{{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    ps.pushRow(points);
    EXPECT_EQ(ps(0, 0), cv::Vec3i(1, 1, 1));
    EXPECT_EQ(ps(0, 1), cv::Vec3i(2, 2, 2));
    EXPECT_EQ(ps(0, 2), cv::Vec3i(3, 3, 3));
}

TEST_F(OrderedPointSet, Fill)
{
    auto ps = vc::OrderedPointSet<cv::Vec3i>::Fill(3, 1, {2, 1, 3});
    EXPECT_EQ(ps(0, 0), cv::Vec3i(2, 1, 3));
    EXPECT_EQ(ps(0, 1), cv::Vec3i(2, 1, 3));
    EXPECT_EQ(ps(0, 2), cv::Vec3i(2, 1, 3));
}

TEST_F(OrderedPointSet, GetRow)
{
    auto row = ps.getRow(0);
    EXPECT_EQ(row[0], cv::Vec3i(1, 1, 1));
    EXPECT_EQ(row[1], cv::Vec3i(2, 2, 2));
    EXPECT_EQ(row[2], cv::Vec3i(3, 3, 3));
}

TEST_F(OrderedPointSet, GetRow_BoundsCheck)
{
    EXPECT_THROW(ps.getRow(-1), std::range_error);
    EXPECT_THROW(ps.getRow(4), std::range_error);
}

TEST_F(OrderedPointSet, CopyRows)
{
    auto newPs = ps.copyRows(0, 3);
    EXPECT_EQ(newPs.width(), 3);
    EXPECT_EQ(newPs.height(), 3);
    EXPECT_EQ(newPs.size(), 9);
    EXPECT_EQ(newPs(0, 0), cv::Vec3i(1, 1, 1));
    EXPECT_EQ(newPs(0, 1), cv::Vec3i(2, 2, 2));
    EXPECT_EQ(newPs(0, 2), cv::Vec3i(3, 3, 3));
    EXPECT_EQ(newPs(1, 0), cv::Vec3i(4, 4, 4));
    EXPECT_EQ(newPs(1, 1), cv::Vec3i(4, 4, 4));
    EXPECT_EQ(newPs(1, 2), cv::Vec3i(4, 4, 4));
    EXPECT_EQ(newPs(2, 0), cv::Vec3i(2, 2, 2));
    EXPECT_EQ(newPs(2, 1), cv::Vec3i(2, 2, 2));
    EXPECT_EQ(newPs(2, 2), cv::Vec3i(2, 2, 2));
}

TEST_F(OrderedPointSet, CopyRows_BadIndexOrder)
{
    EXPECT_THROW(ps.copyRows(3, 2), std::logic_error);
}

TEST_F(OrderedPointSet, CopyRows_BoundsCheck)
{
    EXPECT_THROW(ps.copyRows(0, 6), std::range_error);
}

TEST_F(OrderedPointSet, Append)
{
    auto other = vc::OrderedPointSet<cv::Vec3i>::Fill(3, 4, {1, 3, 5});
    EXPECT_EQ(other.size(), 12);
    EXPECT_EQ(other.width(), 3);
    EXPECT_EQ(other.height(), 4);
    EXPECT_EQ(ps.size(), 12);
    EXPECT_EQ(ps.width(), 3);
    EXPECT_EQ(ps.height(), 4);

    // Append other to ps
    ps.append(other);

    EXPECT_EQ(ps.width(), 3);
    EXPECT_EQ(ps.height(), 8);
    EXPECT_EQ(ps.size(), 24);
    EXPECT_EQ(ps(4, 0), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(4, 1), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(4, 2), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(5, 0), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(5, 1), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(5, 2), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(6, 0), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(6, 1), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(6, 2), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(7, 0), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(7, 1), cv::Vec3i(1, 3, 5));
    EXPECT_EQ(ps(7, 2), cv::Vec3i(1, 3, 5));
}

TEST_F(OrderedPointSet, Append_WidthCheck)
{
    vc::OrderedPointSet<cv::Vec3i> other{4};
    other.pushRow({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}, {4, 4, 4}});
    EXPECT_THROW(ps.append(other), std::logic_error);
}