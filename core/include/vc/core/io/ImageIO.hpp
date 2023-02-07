#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"

namespace volcart
{

/**
 * @brief Read an image from the specified path
 *
 * Currently just a wrapper around:
 *
 * @code
 * cv::imread(path.string(), cv::IMREAD_UNCHANGED);
 * @endcode
 */
auto ReadImage(const filesystem::path& path) -> cv::Mat;

/** WriteImage optional parameters */
struct WriteImageOpts {
    /**
     * Image compression level. Appropriate values depend on the output format.
     */
    std::optional<int> compression;
};

/**
 * @brief Write image to the specified path
 *
 * Uses volcart::WriteTIFF for all tiff images, which includes support for
 * transparency and floating-point images. Otherwise, uses cv::imwrite.
 */
void WriteImage(
    const filesystem::path& path, const cv::Mat& img, WriteImageOpts = {});

}  // namespace volcart
