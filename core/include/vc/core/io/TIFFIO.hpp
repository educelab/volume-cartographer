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

#include <cstdint>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"

namespace volcart::tiffio
{

/** TIFF compression schemes */
enum class Compression : std::uint16_t {
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

/** mmap record */
struct mmap_info {
    /** Whether this is a valid mapping */
    explicit operator bool() const { return addr and size > 0; }
    /** Address of mapped memory */
    void* addr{nullptr};
    /** Size of mapped mempory */
    std::int64_t size{-1};
};

/**
 * @brief Read a TIFF file
 *
 * Reads Gray, Gray+Alpha, RGB, and RGBA TIFF images. Supports 8, 16, and
 * 32-bit integer types as well as 32-bit float types. 3 and 4 channel images
 * will be returned with a BGR channel order, except for 8-bit and 16-bit
 * signed integer types which will be returned with an RGB channel order.
 *
 * Only supports single image TIFF files with scanline encoding and a
 * contiguous planar configuration (this matches the format written by
 * WriteTIFF). Unless you need to read some obscure image type (e.g. 32-bit
 * float or signed integer images), it's generally preferable to use cv::imread.
 *
 * If `mmap_info` is provided, this function will attempt to memory map the
 * TIFF file rather than reading it into memory. If successful, `mmap_info` will
 * contain the address and size required to unmap the file with UnmapTIFF. If
 * memory mapping fails for any reason, this function will fallback to loading
 * the image into memory, and `mmap_info` will be empty. This can be checked
 * with `mmap_info::operator bool()`:
 *
 * ```{.cpp}
 * tiffio::mmap_info info;
 * auto img = tiffio::ReadTIFF("image.tif", &info);
 * if(not info) {
 *   // handle memory mapping failed
 * }
 * ```
 *
 * @warning Memory mapped files will not be unmapped automatically. When done
 * with a memory mapped TIFF, call UnmapTIFF to unmap the file. Failing to do
 * so will result in memory leaks.
 *
 * @note Memory mapping is currently implemented for Linux and macOS systems.
 *
 * @param path Path to TIFF file
 * @param mmap_info mmap_info needed to unmap the file
 * @throws volcart::IOException Unrecoverable read errors
 */
auto ReadTIFF(const filesystem::path& path, mmap_info* mmap_info = nullptr)
    -> cv::Mat;

/**
 * @brief Unmap a memory mapped TIFF file
 *
 * On success, returns 0. If `mmap_info.addr == nullptr`, `mmap_info.size <= 0`,
 * or memory mapping is unsupported by the platform, does nothing and returns
 * -1. If unmapping fails, logs an error and returns a platform-specific error
 * code:
 *  - (Linux/macOS) Returns errno set by munmap.
 */
auto UnmapTIFF(const mmap_info& mmap_info) -> int;

/**
 * @brief Write a TIFF image to file
 *
 * Writes Gray, Gray+Alpha, RGB, and RGBA TIFF images. Supports 8, 16, and
 * 32-bit integer types as well as 32-bit float types. 3 and 4 channel images
 * are assumed to have a BGR channel order, except for 8-bit and 16-bit signed
 * integer types which are not supported.
 *
 * If the raw size of the image (width x height x channels x bytes-per-sample)
 * is >= 4GB, the TIFF will be written using the BigTIFF extension to the TIFF
 * format.
 *
 * @throws volcart::IOException All writing errors
 */
void WriteTIFF(
    const filesystem::path& path,
    const cv::Mat& img,
    Compression compression = Compression::LZW);
}  // namespace volcart::tiffio
