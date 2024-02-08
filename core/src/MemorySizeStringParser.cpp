#include "vc/core/util/MemorySizeStringParser.hpp"

#include <iostream>
#include <regex>
#include <sstream>

static constexpr std::size_t BYTES_PER_KB = 1024;
static constexpr std::size_t BYTES_PER_MB = BYTES_PER_KB * 1024;
static constexpr std::size_t BYTES_PER_GB = BYTES_PER_MB * 1024;
static constexpr std::size_t BYTES_PER_TB = BYTES_PER_GB * 1024;

auto to_string(const float val, const int n = 5) -> std::string;

auto volcart::MemorySizeStringParser(const std::string& s) -> std::size_t
{
    // Regex patterns. Stored as string for easy concatenation.
    std::string numRE{"^[0-9]+"};
    std::string multRE{"(K|M|G|T)?B?$"};

    // Make sure the whole pattern matches
    if (!std::regex_match(s, std::regex{numRE + multRE, std::regex::icase})) {
        throw std::domain_error("Unrecognized size string: " + s);
    }

    // Match the integer value
    std::smatch match;
    std::regex_search(s, match, std::regex{numRE});
    std::size_t value = std::stoull(match[0]);

    // Match the suffix
    std::regex_search(s, match, std::regex{multRE, std::regex::icase});
    std::string suffix = match[1];

    // Return value * multiplier
    // TB
    if (suffix == "T" || suffix == "t") {
        return value * BYTES_PER_TB;
    }

    // GB
    else if (suffix == "G" || suffix == "g") {
        return value * BYTES_PER_GB;
    }

    // MB
    else if (suffix == "M" || suffix == "m") {
        return value * BYTES_PER_MB;
    }

    // KB
    else if (suffix == "K" || suffix == "k") {
        return value * BYTES_PER_KB;
    }

    // Bytes
    else {
        return value;
    }
}

auto volcart::BytesToMemorySizeString(
    std::size_t bytes, const std::string& suffix, MemoryStringFormat fmt)
    -> std::string
{
    // Return value / multiplier
    // TB
    if (suffix == "TB" || suffix == "T" || suffix == "t") {
        if (fmt == MemoryStringFormat::Float) {
            return to_string(static_cast<float>(bytes) / BYTES_PER_TB) + "TB";
        } else {
            return std::to_string(bytes / BYTES_PER_TB) + "TB";
        }
    }

    // GB
    else if (suffix == "GB" || suffix == "G" || suffix == "g") {
        if (fmt == MemoryStringFormat::Float) {
            return to_string(static_cast<float>(bytes) / BYTES_PER_GB) + "GB";
        } else {
            return std::to_string(bytes / BYTES_PER_GB) + "GB";
        }
    }

    // MB
    else if (suffix == "MB" || suffix == "M" || suffix == "m") {
        if (fmt == MemoryStringFormat::Float) {
            return to_string(static_cast<float>(bytes) / BYTES_PER_MB) + "MB";
        } else {
            return std::to_string(bytes / BYTES_PER_MB) + "MB";
        }
    }

    // KB
    else if (suffix == "KB" || suffix == "K" || suffix == "k") {
        if (fmt == MemoryStringFormat::Float) {
            return to_string(static_cast<float>(bytes) / BYTES_PER_KB) + "KB";
        } else {
            return std::to_string(bytes / BYTES_PER_KB) + "KB";
        }
    }

    else if (suffix == "B" || suffix == "b") {
        return std::to_string(bytes) + "B";
    }

    else {
        throw std::domain_error("Unrecognized suffix: " + suffix);
    }
}

auto to_string(const float val, const int n) -> std::string
{
    std::ostringstream out;
    out.precision(n);
    out << val;
    return out.str();
}