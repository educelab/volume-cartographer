#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace volcart
{
/**
 * @brief Convert image to specified depth using max scaling
 *
 * @ingroup Util
 */
cv::Mat QuantizeImage(const cv::Mat& m, int depth = CV_16U);

/**
 * @brief Convert image to specified number of channels
 *
 * @ingroup Util
 */
cv::Mat ColorConvertImage(const cv::Mat& m, int channels = 1);

}  // namespace volcart
