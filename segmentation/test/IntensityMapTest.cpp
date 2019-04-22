#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

#include "vc/segmentation/lrps/IntensityMap.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart::segmentation;

// Global float comparison percent tolerance
static const double perc = 0.01;  // %

// Width/height of reslice window
static const size_t WIDTH = 32;
static const size_t HEIGHT = 32;

cv::Mat_<uint16_t> makeLinearIntensityProfile(
    const std::vector<std::pair<size_t, uint16_t>> maxima);

// Fixture to generate a reslice
class ResliceFixture : public ::testing::Test
{
public:
    cv::Mat_<uint16_t> _reslice;
    ResliceFixture() : _reslice(HEIGHT, WIDTH, uint16_t(0)) {}
};

////////////////////////////////////////////////////////////////////////////////
// Tests with a constant intensity profile

TEST_F(ResliceFixture, AllMaxima)
{
    std::vector<uint16_t> rowVec{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    cv::Mat_<uint16_t> row = cv::Mat_<uint16_t>(rowVec).t();
    row *= 1000;
    row.copyTo(_reslice.row(HEIGHT / 2 + 2));

    IntensityMap map(_reslice, 1, 50, false);
    auto result = map.sortedMaxima();
    EXPECT_EQ(result.size(), map.peakRadius() * 2 + 1);
}

////////////////////////////////////////////////////////////////////////////////
// Tests with a single maxima scaled and shifted

TEST_F(ResliceFixture, OneMaximaInTheMiddle)
{
    // Make a triangle-shaped row with maxima in the center
    std::vector<uint16_t> rowVec{0,  0,  0,  0,  0,  0,  0,  2,  4,  8,  10,
                                 12, 14, 16, 18, 20, 32, 20, 18, 16, 14, 12,
                                 10, 8,  4,  2,  0,  0,  0,  0,  0,  0};
    cv::Mat_<uint16_t> row = cv::Mat_<uint16_t>(rowVec).t();

    // Scale it a bit so normalization/histo equalization doesn't flatten
    // values
    row *= 1000;
    row.copyTo(_reslice.row(HEIGHT / 2 + 1));

    IntensityMap map(_reslice, 1, 50, false);
    auto result = map.sortedMaxima();

    ASSERT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].first, WIDTH / 2);
    volcart::testing::ExpectNear(result[0].second, 1, perc);
}

TEST_F(ResliceFixture, OneMaximaShifted)
{
    // Make a triangle-shaped row with maxima in at center + 5
    std::vector<uint16_t> rowVec{0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                                 0,  2,  4,  8,  10, 12, 14, 16, 18, 20, 32,
                                 20, 18, 16, 14, 12, 10, 8,  4,  2,  0};
    cv::Mat_<uint16_t> row = cv::Mat_<uint16_t>(rowVec).t();

    // Scale it a bit so normalization/histo equalization doesn't flatten values
    row *= 1000;
    row.copyTo(_reslice.row(HEIGHT / 2 + 1));

    IntensityMap map(_reslice, 1, 50, false);
    auto result = map.sortedMaxima();

    ASSERT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].first, WIDTH / 2 + 5);
    volcart::testing::ExpectNear(result[0].second, 1, perc);
}

////////////////////////////////////////////////////////////////////////////////
// Tests with two maxima

TEST_F(ResliceFixture, TwoMaximaEquallySpacedInPeakRadius)
{
    std::vector<uint16_t> rowVec{0,  0,  0,  0,  2,  4, 8,  10, 12, 14, 16,
                                 18, 20, 32, 20, 10, 0, 10, 20, 32, 20, 18,
                                 16, 14, 12, 10, 8,  4, 2,  0,  0,  0};
    cv::Mat_<uint16_t> row = cv::Mat_<uint16_t>(rowVec).t();
    row *= 1000;
    row.copyTo(_reslice.row(HEIGHT / 2 + 1));

    IntensityMap map(_reslice, 1, 50, false);
    auto result = map.sortedMaxima();

    ASSERT_EQ(result.size(), 2);
    EXPECT_EQ(result[0].first, WIDTH / 2 - 3);
    EXPECT_EQ(result[1].first, WIDTH / 2 + 3);
}

TEST_F(ResliceFixture, TwoMaximaOneInsidePeakRadius)
{
    std::vector<uint16_t> rowVec{0,  0,  0,  0,  2,  4, 8, 10, 12, 14, 16,
                                 18, 20, 32, 20, 14, 8, 2, 0,  2,  8,  14,
                                 20, 24, 32, 16, 8,  4, 2, 1,  0,  0};
    cv::Mat_<uint16_t> row = cv::Mat_<uint16_t>(rowVec).t();
    row *= 1000;
    row.copyTo(_reslice.row(HEIGHT / 2 + 1));

    IntensityMap map(_reslice, 1, 50, false);
    auto result = map.sortedMaxima();

    ASSERT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].first, WIDTH / 2 - 3);
}

TEST_F(ResliceFixture, TwoMaximaOneCloserToMiddle)
{
    std::vector<uint16_t> rowVec{0,  0,  0,  0,  2,  4, 8,  10, 12, 14, 16,
                                 18, 20, 32, 20, 10, 0, 32, 28, 24, 20, 18,
                                 16, 14, 12, 10, 8,  4, 2,  0,  0,  0};
    cv::Mat_<uint16_t> row = cv::Mat_<uint16_t>(rowVec).t();
    row *= 1000;
    row.copyTo(_reslice.row(HEIGHT / 2 + 1));

    IntensityMap map(_reslice, 1, 50, false);
    auto result = map.sortedMaxima();

    ASSERT_EQ(result.size(), 2);
    EXPECT_EQ(result[0].first, WIDTH / 2 + 1);
    EXPECT_EQ(result[1].first, WIDTH / 2 - 3);
}

TEST_F(ResliceFixture, TwoMaximaOneCloserToMiddleButShorter)
{
    std::vector<uint16_t> rowVec{0,  0,  0,  0,  2,  4, 8, 10, 12, 14, 16,
                                 18, 20, 32, 20, 10, 0, 5, 14, 12, 10, 8,
                                 4,  2,  0,  0,  0,  0, 0, 0,  0,  0};
    cv::Mat_<uint16_t> row = cv::Mat_<uint16_t>(rowVec).t();
    row *= 1000;
    row.copyTo(_reslice.row(HEIGHT / 2 + 1));

    IntensityMap map(_reslice, 1, 50, false);
    auto result = map.sortedMaxima();

    ASSERT_EQ(result.size(), 2);
    EXPECT_EQ(result[0].first, WIDTH / 2 - 3);
    EXPECT_EQ(result[1].first, WIDTH / 2 + 2);
}