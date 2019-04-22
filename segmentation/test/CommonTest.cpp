#include <gtest/gtest.h>

#include <iostream>
#include <numeric>

#include <opencv2/core.hpp>

#include "vc/segmentation/lrps/Common.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart::segmentation;

static const double floatComparePercentTolerance = 0.01;  // %

////////////////////////////////////////////////////////////////////////////////
// Zip() testing
// Note: explicitly not testing different sizes since an assert is generated.
// This is a programmer error, not a runtime error, so it doesn't make much
// sense to unit test

TEST(CommonTest, ZipTestingSizeEmpty)
{
    std::vector<int> v1, v2;
    auto pairs = Zip(v1, v2);
    EXPECT_TRUE(pairs.empty());
}

TEST(CommonTest, ZipTestingSizeOne)
{
    std::vector<int> v1{0};
    std::vector<int> v2{0};
    auto pairs = Zip(v1, v2);
    EXPECT_EQ(pairs.size(), 1);
    std::pair<int, int> zeroPair(0, 0);
    EXPECT_EQ(pairs[0].first, zeroPair.first);
    EXPECT_EQ(pairs[0].second, zeroPair.second);
}

TEST(CommonTest, ZipTestingDifferentTypes)
{
    std::vector<int> v1{1};
    std::vector<double> v2{1.0};
    auto pairs = Zip(v1, v2);
    EXPECT_EQ(pairs.size(), 1);
    std::pair<int, double> mixedPair(1, 1);
    EXPECT_EQ(pairs[0].first, mixedPair.first);
    EXPECT_EQ(pairs[0].second, mixedPair.second);
}

////////////////////////////////////////////////////////////////////////////////
// Unzip() testing

TEST(CommonTest, UnzipTestingSizeEmpty)
{
    std::vector<cv::Vec3d> vs;
    std::vector<double> xs, ys;
    std::tie(xs, ys) = Unzip(vs);
    EXPECT_TRUE(xs.empty());
    EXPECT_TRUE(ys.empty());
}

TEST(CommonTest, UnzipTestingSizeOne)
{
    std::vector<cv::Vec3d> vs{{0, 0, 0}};
    std::vector<double> xs, ys;
    std::tie(xs, ys) = Unzip(vs);
    EXPECT_EQ(xs.size(), 1);
    EXPECT_EQ(ys.size(), 1);
    EXPECT_EQ(xs[0], 0);
    EXPECT_EQ(ys[0], 0);
}

TEST(CommonTest, UnzipTestingSizeTwo)
{
    std::vector<cv::Vec3d> vs{{0, 0, 0}, {1, 1, 1}};
    std::vector<double> xs, ys;
    std::tie(xs, ys) = Unzip(vs);
    EXPECT_EQ(xs.size(), vs.size());
    EXPECT_EQ(ys.size(), vs.size());
    for (size_t i = 0; i < vs.size(); ++i) {
        EXPECT_EQ(xs[i], vs[i](0));
        EXPECT_EQ(ys[i], vs[i](1));
    }
}

////////////////////////////////////////////////////////////////////////////////
// NormalizeVector(built-in types) testing

TEST(CommonTest, NormalizeVectorSizeEmpty)
{
    std::vector<int> is;
    auto ds = NormalizeVector(is);
    EXPECT_TRUE(ds.empty());
}

TEST(CommonTest, NormalizeVectorSizeOneCheckNormalized)
{
    std::vector<int> is{5};
    auto ds = NormalizeVector(is);
    EXPECT_EQ(ds.size(), 1);
    EXPECT_EQ(ds[0], 1.0);
}

TEST(CommonTest, NormalizeVectorIntegersZeroToNineDefaultMinAndMax)
{
    std::vector<int> is(10);
    std::iota(std::begin(is), std::end(is), 0);
    auto ds = NormalizeVector(is);
    EXPECT_EQ(ds.size(), is.size());

    // sum + delta generates correct values as verified from python library
    // sklearn.processing.MinMaxScaler
    double sum = 0;
    double delta = 0.1111111111111111111;
    for (const auto e : ds) {
        volcart::testing::ExpectNear(e, sum, floatComparePercentTolerance);
        sum += delta;
    }
}

TEST(CommonTest, NormalizeVectorIntegersZeroToNineShiftRangeByFive)
{
    std::vector<int> is(10);
    std::iota(std::begin(is), std::end(is), 0);
    auto newMin = double(is.front() + 5);
    auto newMax = double(is.back() + 5);
    auto ds = NormalizeVector(is, newMin, newMax);
    EXPECT_EQ(ds.size(), is.size());
    double baseVal = newMin;
    for (const auto e : ds) {
        volcart::testing::ExpectNear(e, baseVal, floatComparePercentTolerance);
        baseVal += 1;
    }
}

TEST(CommonTest, NormalizeVectorIntegersZeroToNineCompressRangeByHalf)
{
    std::vector<int> is(10);
    std::iota(std::begin(is), std::end(is), 0);
    auto newMin = double(is.front());
    auto newMax = is.back() / 2.0;
    auto ds = NormalizeVector(is, newMin, newMax);
    EXPECT_EQ(ds.size(), is.size());
    double baseVal = newMin;
    for (const auto e : ds) {
        volcart::testing::ExpectNear(e, baseVal, floatComparePercentTolerance);
        baseVal += 0.5;
    }
}

////////////////////////////////////////////////////////////////////////////////
// NormalizeVector(cv::Vec<T, Len>) overload testing
// XXX Should more tests be added? This function works on a per-element basis,
// not across the entire vector. If it works for one element, it should work for
// all elements

TEST(CommonTest, NormalizeVectorCvVecSizeEmpty)
{
    std::vector<cv::Vec3i> is;
    auto ds = NormalizeVector(is);
    EXPECT_TRUE(ds.empty());
}

TEST(CommonTest, NormalizeVectorCvVecSizeOneCheckNormalized)
{
    std::vector<cv::Vec3i> is{{1, 2, 3}};

    // Computed via Python
    cv::Vec3d expectedResult{0.26726124191, 0.5345224838, 0.801783725737};

    auto ds = NormalizeVector(is);
    EXPECT_EQ(ds.size(), 1);
    for (int i = 0; i < 3; ++i) {
        volcart::testing::ExpectNear(
            ds.front()(i), expectedResult(i), floatComparePercentTolerance);
    }

    // Check that the resulting vector norm is 1
    volcart::testing::ExpectNear(
        cv::norm(ds.front()), 1.0, floatComparePercentTolerance);
}

////////////////////////////////////////////////////////////////////////////////
// SquareDiff() testing

TEST(CommonTest, SquareDiffTestsSizeEmpty)
{
    std::vector<cv::Vec3d> v1, v2;
    auto testResult = SquareDiff(v1, v2);
    EXPECT_TRUE(testResult.empty());
}

TEST(CommonTest, SquareDiffTestsSizeOne)
{
    std::vector<cv::Vec3d> v1{{0, 0, 0}}, v2{{1, 1, 1}};
    auto testResult = SquareDiff(v1, v2);
    EXPECT_EQ(testResult.size(), 1);
    double expectedResult = std::sqrt(3);
    volcart::testing::ExpectNear(
        testResult[0], expectedResult, floatComparePercentTolerance);
}

////////////////////////////////////////////////////////////////////////////////
// SumSquareDiff() testing

TEST(CommonTest, SumSquareDiffTestsSizeEmpty)
{
    std::vector<double> v1, v2;
    auto testResult = SumSquareDiff(v1, v2);
    EXPECT_EQ(testResult, 0.0);
}

TEST(CommonTest, SumSquareDiffTestsSizeOne)
{
    std::vector<double> v1{0}, v2{1};
    double testResult = SumSquareDiff(v1, v2);
    double expectedResult = 1.0;
    volcart::testing::ExpectNear(
        testResult, expectedResult, floatComparePercentTolerance);
}

TEST(CommonTest, SumSquareDiffTestsSizeTwo)
{
    std::vector<double> v1{0, 1}, v2{1, 0};
    double testResult = SumSquareDiff(v1, v2);
    double expectedResult = std::sqrt(2);
    volcart::testing::ExpectNear(
        testResult, expectedResult, floatComparePercentTolerance);
}