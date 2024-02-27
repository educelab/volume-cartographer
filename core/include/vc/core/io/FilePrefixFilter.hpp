#pragma once

/** @file */

#include <regex>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "vc/core/filesystem.hpp"

namespace volcart
{
namespace io
{

/**
 * Returns true if `path` has a prefix beginning with `prefix`. Comparison is
 * case insensitive.
 */
inline bool FilePrefixFilter(
    const volcart::filesystem::path& path, const std::string& prefix)
{
    return boost::algorithm::to_lower_copy(path.filename().string())
               .rfind(boost::algorithm::to_lower_copy(prefix), 0) == 0;
}

}  // namespace io

/** Convenience alias for FilePrefixFilter */
constexpr auto IsFilePrefix = io::FilePrefixFilter;

}  // namespace volcart
