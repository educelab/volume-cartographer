#pragma once

/** @file */

#include <cstddef>

#include <opencv2/core.hpp>

namespace volcart
{

/**
 * @brief Apply a LUT to an image
 *
 * Maps the pixel values of an image to the range of a LUT. LUT must be a
 * cv::Mat with shape \f$(1, bins)\f$. Input images should be `CV_8U`, `CV_16U`,
 * or `CV_32F`. Inputs with more than one channel will be converted to grayscale
 * prior to mapping. Pixel values \f$\leq\f$ min will be mapped to the first bin
 * and values \f$\geq\f$ max will be mapped to the last bin. If invert is true,
 * the bin mapping will be reversed.
 *
 * @ingroup Util
 */
cv::Mat ApplyLUT(
    const cv::Mat& img,
    const cv::Mat& lut,
    float min,
    float max,
    bool invert = false);

/**
 * @brief Apply a LUT to an image
 *
 * Maps the pixel values of an image to the range of a LUT. LUT must be a
 * cv::Mat with shape \f$(1, bins)\f$. Input images should be `CV_8U`, `CV_16U`,
 * or `CV_32F`. Inputs with more than one channel will be converted to grayscale
 * prior to mapping.
 *
 * This overload provides two linear maps. Pixel values in the range
 * \f$[min, mid)f$ will be mapped to the bin range \f$[0, bins / 2)f$ and
 * values in the range \f$[mid, max)f$ will be mapped to the bin range
 * \f$[bins / 2, bins)f$. This is useful if you want to map a specific value to
 * the midpoint of a LUT (e.g. ColorMap::BWR). If invert is true, the bin
 * mapping will be reversed.
 */
cv::Mat ApplyLUT(
    const cv::Mat& img,
    const cv::Mat& lut,
    float min,
    float mid,
    float max,
    bool invert = false);

/**
 * @brief Apply a LUT to an image
 *
 * Maps the pixel values of an image to the range of a LUT. LUT must be a
 * cv::Mat with shape \f$(1, bins)\f$. Input images should be `CV_8U`, `CV_16U`,
 * or `CV_32F`. Inputs with more than one channel will be converted to grayscale
 * prior to mapping. If invert is true, the bin mapping will be reversed.
 *
 * This overload calculates the range of data values and maps that to the range
 * of the LUT. A mask image may be provided to control the region used for
 * calculating the data min/max.
 *
 * @ingroup Util
 */
cv::Mat ApplyLUT(
    const cv::Mat& img,
    const cv::Mat& lut,
    bool invert = false,
    const cv::Mat& mask = cv::Mat());

/**
 * @brief Generate a LUT scale bar image
 *
 * Generates an image plot of a LUT for use as a scale bar. If output image has
 * a horizontal aspect ratio (width >= height), min to max values will be
 * plotted left to right. If the output image has a vertical aspect ratio, then
 * min to max values will be plotted bottom to top. If invert is true, the bin
 * mapping will be reversed. If experiencing color banding, consider increasing
 * the number of bins used when calling GetColorMap.
 *
 * @ingroup Util
 */
cv::Mat GenerateLUTScaleBar(
    const cv::Mat& lut,
    bool invert = false,
    std::size_t height = 36,
    std::size_t width = 256);

}  // namespace volcart
