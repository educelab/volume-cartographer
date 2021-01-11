#pragma once

/** @file */

#include <regex>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

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
inline bool FileExtensionFilter(
    const boost::filesystem::path& path, const ExtensionList& exts)
{
    std::string regexExpression = ".*\\.(";
    size_t count = 0;
    for (const auto& e : exts) {
        regexExpression.append(e);
        if (++count < exts.size()) {
            regexExpression.append("|");
        }
    }
    regexExpression.append(")$");

    std::regex extensions{regexExpression, std::regex::icase};
    return std::regex_match(path.extension().string(), extensions);
}

}  // namespace io

/** Convenience alias for FileExtensionFilter */
constexpr auto IsFileType = io::FileExtensionFilter;

}  // namespace volcart
