#pragma once

/** @file */

#include <cstdint>

#include "vc/core/filesystem.hpp"

namespace volcart
{

namespace endian
{
/** Returns whether the system is little endian */
auto little() -> bool;

/** Returns whether the system is big endian */
auto big() -> bool;
}  // namespace endian

/** @brief Memmap record */
struct mmap_info {
    /** Whether this is a valid mapping */
    explicit operator bool() const;
    /** Address of mapped memory */
    void* addr{nullptr};
    /** Size of mapped memory */
    std::int64_t size{-1};
};

/**
 * @brief Memmap record which automatically unmaps the file on destruction
 *
 * The same as mmap_info but automatically calls UnmapFile() on destruction.
 * To avoid unintended unmapping, this class cannot be copied but can be moved.
 */
struct auto_mmap_info : mmap_info {
    /** Default constructor */
    auto_mmap_info() = default;

    /** Copy construct from mmap_info */
    explicit auto_mmap_info(const mmap_info& rhs);
    /** Move construct from mmap_info */
    explicit auto_mmap_info(mmap_info&& rhs);
    /** Copy assign from mmap_info */
    auto operator=(const mmap_info& rhs) -> auto_mmap_info&;
    /** Move assign from mmap_info */
    auto operator=(mmap_info&& rhs) -> auto_mmap_info&;

    /** Move assign from another auto_mmap_info */
    auto operator=(auto_mmap_info&& rhs) noexcept -> auto_mmap_info&;

    /** Cannot copy construct from another auto_mmap_info */
    explicit auto_mmap_info(auto_mmap_info&) = delete;
    /** Cannot copy construct from another auto_mmap_info */
    explicit auto_mmap_info(const auto_mmap_info&) = delete;
    /** Cannot copy assign from another auto_mmap_info */
    auto operator=(auto_mmap_info&) -> auto_mmap_info& = delete;
    /** Cannot copy construct from another auto_mmap_info */
    auto operator=(const auto_mmap_info&) -> auto_mmap_info& = delete;

    /** Auto-unmapping destructor */
    ~auto_mmap_info();
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
auto memmap_supported() -> bool;
}  // namespace volcart
