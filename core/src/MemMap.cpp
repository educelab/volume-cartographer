#include "vc/core/util/MemMap.hpp"

#include "vc/core/util/Logging.hpp"

namespace vc = volcart;
namespace fs = vc::filesystem;

vc::mmap_info::operator bool() const { return addr and size > 0; }

vc::auto_mmap_info::auto_mmap_info(const mmap_info& rhs) : mmap_info(rhs) {}

vc::auto_mmap_info::auto_mmap_info(mmap_info&& rhs) : mmap_info(rhs)
{
    rhs.addr = nullptr;
    rhs.size = -1;
}

auto vc::auto_mmap_info::operator=(const mmap_info& rhs) -> auto_mmap_info&
{
    mmap_info::operator=(rhs);
    return *this;
}

auto vc::auto_mmap_info::operator=(mmap_info&& rhs) -> auto_mmap_info&
{
    mmap_info::operator=(rhs);
    return *this;
}

auto vc::auto_mmap_info::operator=(auto_mmap_info&& rhs) noexcept
    -> auto_mmap_info&
{
    addr = rhs.addr;
    size = rhs.size;
    rhs.addr = nullptr;
    rhs.size = -1;
    return *this;
}

vc::auto_mmap_info::~auto_mmap_info()
{
    if (*this) {
        UnmapFile(*this);
    }
}

auto vc::memmap_supported() -> bool { return VC_MEMMAP_SUPPORTED; }

///// Platform-specific memory mapping /////
// Linux/macOS
#if defined(VC_MEMMAP_MMAP)
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

auto vc::MemmapFile(const fs::path& path) -> mmap_info
{
    Logger()->trace("Memory mapping file: {}", path.string());

    // Open file descriptor
    const int fd = open(path.c_str(), O_RDONLY);
    if (fd == -1) {
        Logger()->debug(
            "Failed to open file descriptor: {}. {}", path.string(),
            std::strerror(errno));
        return {};
    }

    // Get file stats (namely size)
    struct stat sb {
    };
    if (fstat(fd, &sb) == -1) {
        Logger()->debug(
            "Failed to fstat: {}. {}", path.string(), std::strerror(errno));
        close(fd);
        return {};
    }

    // Memory map the file
    auto* data = mmap(nullptr, sb.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) {
        Logger()->debug(
            "Failed to mmap: {}. {}", path.string(), std::strerror(errno));
        close(fd);
        return {};
    }
    // Descriptor no longer needed
    close(fd);

    return {.addr = data, .size = sb.st_size};
}

auto vc::UnmapFile(mmap_info& mmap_info) -> int
{
    if (not mmap_info.addr) {
        Logger()->debug("Empty address");
        return -1;
    }
    if (mmap_info.size < 1) {
        Logger()->debug("Invalid mapping size: {}", mmap_info.size);
        return -1;
    }

    const auto err = munmap(mmap_info.addr, mmap_info.size);
    int res{0};
    if (err == -1) {
        res = errno;
        Logger()->error("Failed to unmap file: {}", std::strerror(res));
    }
    mmap_info.addr = nullptr;
    mmap_info.size = -1;
    return res;
}

#else
// All unsupported platforms
#pragma message("Memory mapping is not implemented on this plaform")
auto vc::MemmapFile(const fs::path& path) -> mmap_info { return {}; }
auto vc::UnmapFile(mmap_info& mmap_info) -> int { return -1; }
#endif