#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
#pragma once

/** @file */

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace volcart
{

struct CannySettings {
    int blurSize = 1;
    int closingSize = 4;
    int apertureSize = 0;
    int minThreshold = 100;
    int maxThreshold = 150;
    bool contour = false;
    bool bilateral = false;
    bool midpoint = false;
    char projectionFrom = 'L';
    std::vector<std::string> fromMeshes = {};
    cv::Mat mask;
    std::size_t zMin = 0;
    std::size_t zMax = 1;
};

/**
 * @brief Perform Canny edge segmentation on an image
 *
 * @ingroup Util
 */
auto Canny(cv::Mat src, CannySettings settings) -> cv::Mat;

}  // namespace volcart

#pragma clang diagnostic pop