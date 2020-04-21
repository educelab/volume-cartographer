/**
 * @file TIFFIO.hpp
 * @brief IO Utilities for TIFF files
 * @author Seth Parker
 * @date 4/14/17
 *
 * @ingroup IO
 */

#pragma once

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

namespace volcart
{
namespace tiffio
{
/**
 * @brief Write a TIFF image to file
 *
 * Supports writing floating point and signed integer TIFFs, in addition to
 * unsigned 8 & 16 bit integer types. Currently only supports single and
 * three-channel images. Unless you need to write one of the special types of
 * images, using cv::imwrite() is a better option.
 */
void WriteTIFF(const boost::filesystem::path& path, const cv::Mat& img);
}  // namespace tiffio
}  // namespace volcart
