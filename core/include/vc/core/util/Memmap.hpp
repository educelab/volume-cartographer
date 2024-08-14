#pragma once

#include <cstdint>

#include "vc/core/filesystem.hpp"

namespace volcart
{

/** mmap record */
struct mmap_info {
    /** Whether this is a valid mapping */
    explicit operator bool() const;
    /** Address of mapped memory */
    void* addr{nullptr};
    /** Size of mapped mempory */
    std::int64_t size{-1};
};

/**
 * @brief Memory map a file
 *
 * If successful, `mmap_info` will contain the address and size required to
 * unmap the file with UnmapFile. If memory mapping fails for any reason,
 * `mmap_info` will be empty. This can be checked with
 * `mmap_info::operator bool()`:
 *
 * ```{.cpp}
 * auto info = MemmapFile("image.txt");
 * if(not info) {
 *   // handle memory mapping failed
 * }
 * ```
 */
auto MemmapFile(const filesystem::path& path) -> mmap_info;

/**
 * @brief Unmap a memory mapped file
 *
 * On success, returns 0. If `mmap_info.addr == nullptr`, `mmap_info.size <= 0`,
 * or memory mapping is unsupported by the platform, does nothing and returns
 * -1. If unmapping fails, logs an error and returns a platform-specific error
 * code:
 *  - (Linux/macOS) Returns errno set by munmap.
 */
auto UnmapFile(mmap_info& mmap_info) -> int;

/** Whether memory mapping is available on this platform */
// TODO: Define this macro using CMake
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#define VC_MEMMAP_SUPPORTED
constexpr auto MEMMAP_SUPPORTED = true;
#else
constexpr auto MEMMAP_SUPPORTED = false;
#endif
}  // namespace volcart
