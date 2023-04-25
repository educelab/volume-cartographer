#pragma once

/** @file */

#ifdef VC_USE_BOOSTFS
#include <boost/filesystem.hpp>
namespace volcart
{
/**
 * @namespace volcart::filesystem
 * @brief Alias for `boost::filesystem`
 *
 * If CMake variable `VC_USE_BOOSTFS` is false, then `VC_FS_LIB` is an
 * alias for `std::filesystem`. Otherwise, it is an alias for
 * `boost::filesystem`. This enables filesystem compatibility for compilers
 * which do not provide `std::filesystem`.
 */
namespace filesystem = boost::filesystem;
}  // namespace volcart
#else
#include <filesystem>
namespace volcart
{
/**
 * @namespace volcart::filesystem
 * @brief Alias for `std::filesystem`
 *
 * If CMake variable `VC_USE_BOOSTFS` is false, then `VC_FS_LIB` is an
 * alias for `std::filesystem`. Otherwise, it is an alias for
 * `boost::filesystem`. This enables filesystem compatibility for compilers
 * which do not provide `std::filesystem`.
 */
namespace filesystem = std::filesystem;
}  // namespace volcart
#endif