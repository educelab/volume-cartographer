#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/String.hpp"

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

    /** Manually specified padding when using WriteImageSequence */
    std::optional<int> padding;
};

/**
 * @brief Write image to the specified path
 *
 * Uses volcart::WriteTIFF for all tiff images, which includes support for
 * transparency and floating-point images. Otherwise, uses cv::imwrite.
 *
 * @throws volcart::IOException
 */
void WriteImage(
    const filesystem::path& path, const cv::Mat& img, WriteImageOpts = {});

template <class Iterable>
void WriteImageSequence(
    const filesystem::path& path,
    const Iterable& iterable,
    const WriteImageOpts& opts = {})
{
    namespace fs = volcart::filesystem;

    // components
    fs::path parent;
    std::string prefix;
    std::string suffix;
    fs::path ext;

    // If directory, default to dir/###.tif
    if (fs::is_directory(path)) {
        parent = path;
        ext = ".tif";
    }

    // If path, decompose to replace {} with a number
    else {
        parent = path.parent_path();
        ext = path.extension();

        // Split into a prefix and suffix
        auto stem = path.stem().string();
        std::tie(prefix, std::ignore, suffix) = partition(stem, "{}");

        // Log when separator not found
        if (suffix.empty()) {
            Logger()->debug(
                "Index placement separator \\{\\} not found in stem: {}", stem);
        }
    }

    // Setup padding (default or user-provided)
    auto pad = std::to_string(std::size(iterable)).size();
    pad = opts.padding.value_or(pad);

    // Write images
    for (const auto [i, image] : enumerate(iterable)) {
        const auto name = prefix + to_padded_string(i, pad) + suffix;
        auto filepath = (parent / name).replace_extension(ext);
        WriteImage(filepath, image, opts);
    }
}

}  // namespace volcart
