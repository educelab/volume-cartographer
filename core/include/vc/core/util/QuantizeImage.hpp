#pragma once

#include <limits>

#include <opencv2/core.hpp>

namespace volcart
{
/**
 * @brief Convert image to specified depth using max scaling
 *
 * @ingroup Util
 */
inline cv::Mat QuantizeImage(const cv::Mat& m, int depth = CV_16U)
{
    // Copy to output
    auto output = m.clone();

    // Setup the max value for integer images
    double outputMax{1.0};
    switch (depth) {
        case CV_8U:
            outputMax = std::numeric_limits<uint8_t>::max();
            break;
        case CV_8S:
            outputMax = std::numeric_limits<int8_t>::max();
            break;
        case CV_16U:
            outputMax = std::numeric_limits<uint16_t>::max();
            break;
        case CV_16S:
            outputMax = std::numeric_limits<int16_t>::max();
            break;
        default:
            // do nothing
            break;
    }

    // Scale to output
    double min, max;
    cv::minMaxLoc(output, &min, &max);
    output.convertTo(
        output, depth, outputMax / (max - min), -min * outputMax / (max - min));

    return output;
}
}  // namespace volcart
