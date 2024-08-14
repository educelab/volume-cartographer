#pragma once

#include <cstdint>

namespace volcart
{

/** mmap record */
struct mmap_info {
    /** Whether this is a valid mapping */
    explicit operator bool() const { return addr and size > 0; }
    /** Address of mapped memory */
    void* addr{nullptr};
    /** Size of mapped mempory */
    std::int64_t size{-1};
};

}  // namespace volcart
