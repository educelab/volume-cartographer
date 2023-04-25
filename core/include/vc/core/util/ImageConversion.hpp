#pragma once

/** @file */

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace volcart
{

/** @brief Convert an OpenCV depth enum value to a human-readable string */
auto DepthToString(int depth) -> std::string;

/**
 * @brief Convert image to specified depth using max scaling
 *
 * If `scaleMinMax == true`, the input min and max will be calculated and scaled
 * the full range of the output depth. Otherwise, the range of the input depth
 * will be scaled to the range of the output depth. For floating-point images,
 * the default input range is assumed to be [0,1].
 *
 * @ingroup Util
 */
auto QuantizeImage(
    const cv::Mat& m, int depth = CV_16U, bool scaleMinMax = true) -> cv::Mat;

/**
 * @brief Convert image to specified number of channels
 *
 * @ingroup Util
 */
auto ColorConvertImage(const cv::Mat& m, int channels = 1) -> cv::Mat;

}  // namespace volcart
