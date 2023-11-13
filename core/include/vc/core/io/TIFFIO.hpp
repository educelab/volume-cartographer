/**
 * @file
 *
 * @brief IO Utilities for TIFF files
 * @author Seth Parker
 * @date 4/14/17
 *
 * @ingroup IO
 */

#pragma once

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"

namespace volcart::tiffio
{

/** TIFF compression schemes */
enum class Compression {
    NONE = 1,
    CCITTRLE = 2,
    CCITTFAX3 = 3,
    CCITT_T4 = 3,
    CCITTFAX4 = 4,
    CCITT_T6 = 4,
    LZW = 5,
    OJPEG = 6,
    JPEG = 7,
    ADOBE_DEFLATE = 8,
    NEXT = 32766,
    CCITTRLEW = 32771,
    PACKBITS = 32773,
    THUNDERSCAN = 32809,
    IT8CTPAD = 32895,
    IT8LW = 32896,
    IT8MP = 32897,
    IT8BL = 32898,
    PIXARFILM = 32908,
    PIXARLOG = 32909,
    DEFLATE = 32946,
    DCS = 32947,
    JBIG = 34661,
    SGILOG = 34676,
    SGILOG24 = 34677,
    JP2000 = 34712
};

/**
 * @brief Read a TIFF file
 *
 * Supports 8-bit and 16-bit integer types as well as 32-bit float types. This
 * only supports single image TIFF files, so it's generally preferable to use
 * something like cv::imread in most cases.
 *
 * @param path Path to TIFF file
 */
auto ReadTIFF(const volcart::filesystem::path& path) -> cv::Mat;

/**
 * @brief Write a TIFF image to file
 *
 * Supports writing floating point and signed integer TIFFs, in addition to
 * unsigned 8 & 16 bit integer types. Also supports 1-4 channel images.
 */
void WriteTIFF(
    const volcart::filesystem::path& path,
    const cv::Mat& img,
    Compression compression = Compression::LZW);
}  // namespace volcart::tiffio
