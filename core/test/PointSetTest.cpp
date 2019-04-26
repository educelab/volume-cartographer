#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "vc/core/types/PointSet.hpp"

using namespace volcart;

class Vec3iPointSet : public ::testing::Test
{
public:
    PointSet<cv::Vec3i> ps;

    Vec3iPointSet() : ps(3)
    {
        ps.push_back({1, 1, 1});
        ps.push_back({2, 2, 2});
        ps.push_back({3, 3, 3});
    }
};

TEST(PointSetTest, EmptyPointSetTest)
{
    PointSet<cv::Vec3i> ps;
    EXPECT_EQ(ps.size(), 0);
    EXPECT_TRUE(ps.empty());
}

TEST_F(Vec3iPointSet, OneRowPointSetTest)
{
    EXPECT_EQ(ps.size(), 3);
    EXPECT_TRUE(!ps.empty());
}

TEST_F(Vec3iPointSet, OneRowPointSetIteratorTest)
{
    EXPECT_EQ(ps.front(), cv::Vec3i(1, 1, 1));
    EXPECT_EQ(ps.back(), cv::Vec3i(3, 3, 3));
    EXPECT_EQ(*std::begin(ps), cv::Vec3i(1, 1, 1));
    EXPECT_EQ(*(std::end(ps) - 1), cv::Vec3i(3, 3, 3));
    size_t i = 1;
    for (auto p : ps) {
        EXPECT_EQ(p, cv::Vec3i(i, i, i));
        ++i;
    }
}

TEST(PointSetTest, FillPointSetTest)
{
    PointSet<cv::Vec3i> ps(3, {1, 1, 1});
    EXPECT_EQ(ps.size(), 3);
    for (auto p : ps) {
        EXPECT_EQ(p, cv::Vec3i(1, 1, 1));
    }
}

TEST_F(Vec3iPointSet, ClearPointSetTest)
{
    ps.clear();
    EXPECT_EQ(ps.size(), 0);
}

TEST_F(Vec3iPointSet, StatisticsPointSetTest)
{
    EXPECT_EQ(ps.min(), cv::Vec3i(1, 1, 1));
    EXPECT_EQ(ps.max(), cv::Vec3i(3, 3, 3));
    auto minmax = ps.minMax();
    EXPECT_EQ(minmax.first, cv::Vec3i(1, 1, 1));
    EXPECT_EQ(minmax.second, cv::Vec3i(3, 3, 3));
}

TEST(PointSetTest, StatisticsEmptyPointSet)
{
    PointSet<cv::Vec3i> ps;
    EXPECT_THROW(ps.min(), std::range_error);
    EXPECT_THROW(ps.max(), std::range_error);
    EXPECT_THROW(ps.minMax(), std::range_error);
}

TEST_F(Vec3iPointSet, AppendPointSetToAnother)
{
    PointSet<cv::Vec3i> other(4);
    other.push_back({1, 2, 3});
    other.push_back({4, 5, 6});
    other.push_back({7, 8, 9});
    EXPECT_EQ(other.size(), 3);
    EXPECT_EQ(ps.size(), 3);

    // Append new pointset to old
    ps.append(other);

    EXPECT_EQ(ps.size(), 6);
    EXPECT_EQ(ps[3], cv::Vec3i(1, 2, 3));
    EXPECT_EQ(ps[4], cv::Vec3i(4, 5, 6));
    EXPECT_EQ(ps[5], cv::Vec3i(7, 8, 9));
}

TEST_F(Vec3iPointSet, AppendFullPointSetToFullPointSet)
{
    PointSet<cv::Vec3i> other(3);
    other.push_back({1, 2, 3});
    other.push_back({4, 5, 6});
    other.push_back({7, 8, 9});
    EXPECT_EQ(other.size(), 3);
    EXPECT_EQ(ps.size(), 3);

    // Append new pointset to old
    ps.append(other);

    EXPECT_EQ(ps.size(), 6);
    EXPECT_EQ(ps[3], cv::Vec3i(1, 2, 3));
    EXPECT_EQ(ps[4], cv::Vec3i(4, 5, 6));
    EXPECT_EQ(ps[5], cv::Vec3i(7, 8, 9));
}