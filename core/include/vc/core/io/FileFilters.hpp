#pragma once

/** @file */

#include <regex>
#include <string>
#include <vector>

#include "vc/core/filesystem.hpp"
#include "vc/core/util/String.hpp"

namespace volcart
{
namespace io
{
/** Stores a list of case-insensitive file extensions */
using ExtensionList = std::vector<std::string>;

/**
 * Returns true if `path` has a file extension in `exts`. Comparison is case
 * insensitive.
 */
inline auto FileExtensionFilter(
    const filesystem::path& path, const ExtensionList& exts) -> bool
{
    std::string regexExpression = ".*\\.(";
    std::size_t count = 0;
    for (const auto& e : exts) {
        regexExpression.append(e);
        if (++count < exts.size()) {
            regexExpression.append("|");
        }
    }
    regexExpression.append(")$");

    const std::regex extensions{regexExpression, std::regex::icase};
    return std::regex_match(path.extension().string(), extensions);
}

/**
 * Returns true if `path` is a UNIX hidden file (i.e., the filename starts with
 * the `.` character)
 */
inline auto UnixHiddenFileFilter(const filesystem::path& path) -> bool
{
    return path.filename().string()[0] == '.';
}

}  // namespace io

/** Convenience alias for FileExtensionFilter */
constexpr auto IsFileType = io::FileExtensionFilter;

/** Convenience alias for UnixHiddenFileFilter */
constexpr auto IsUnixHiddenFile = io::UnixHiddenFileFilter;
}  // namespace volcart
