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
/**
 * @brief Write a TIFF image to file
 *
 * Supports writing floating point and signed integer TIFFs, in addition to
 * unsigned 8 & 16 bit integer types. Also supports 1-4 channel images.
 */
void WriteTIFF(const volcart::filesystem::path& path, const cv::Mat& img);
}  // namespace volcart::tiffio
