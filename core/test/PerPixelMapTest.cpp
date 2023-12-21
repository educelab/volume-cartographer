#include <gtest/gtest.h>

#include "vc/core/types/PerPixelMap.hpp"

using namespace volcart;

TEST(PerPixelMap, WriteRead)
{
    // Build a PPM
    PerPixelMap ppm(100, 100);
    cv::Mat mask = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat cellMap = cv::Mat(100, 100, CV_32SC1);
    cellMap = cv::Scalar::all(-1);
    for (auto y = 0; y < 10; ++y) {
        for (auto x = 0; x < 10; ++x) {
            auto outY = 46 + y;
            auto outX = 46 + x;
            auto dx = static_cast<double>(x);
            auto dy = static_cast<double>(y);
            ppm(outY, outX) = {dx, dy, (dx + dy) / 2.0,
                               dx, dy, (dx + dy) / 2.0};
            mask.at<std::uint8_t>(outY, outX) = 255U;
            cellMap.at<std::int32_t>(outY, outX) = y + 10;
        }
    }
    ppm.setMask(mask);
    ppm.setCellMap(cellMap);

    // Write the PPM
    std::string path{"vc_core_PerPixelMap_WriteRead.ppm"};
    EXPECT_NO_THROW(PerPixelMap::WritePPM(path, ppm));

    // Read the PPM
    PerPixelMap result;
    EXPECT_NO_THROW(result = PerPixelMap::ReadPPM(path));

    // Test the values
    for (auto y = 0; y < 100; ++y) {
        for (auto x = 0; x < 100; ++x) {
            EXPECT_EQ(result(y, x), ppm(y, x));
        }
    }

    // Test the mask
    cv::Mat diff = ppm.mask() != result.mask();
    EXPECT_EQ(cv::countNonZero(diff), 0);

    // Test the cell map
    diff = ppm.cellMap() != result.cellMap();
    EXPECT_EQ(cv::countNonZero(diff), 0);
}