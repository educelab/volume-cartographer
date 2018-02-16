/**
 * @file MemorySizeStringParser.hpp
 * @author Seth Parker
 * @date 2/15/18
 *
 * @ingroup Util
 */
#pragma once

#include <string>

namespace volcart
{
/**
 * @brief Converts strings describing memory sizes to byte values
 *
 * Takes a string of the form `[integer](K|M|G|T)B` and returns the number
 * of bytes described by the string. Values without a character suffix are
 * assumed to be bytes.
 *
 * Examples:
 * Input: "1", Output: 1
 * Input: "1KB", Output: 1024
 * Input: "1K", Output: 1024
 * Input: "1M", Output: 1048576
 */
size_t MemorySizeStringParser(const std::string& s);

std::string BytesToMemorySizeString(
    size_t bytes, const std::string& suffix = "G");
}
