#pragma once

#include <string>

namespace volcart
{
/** @brief Converts a printf-style format string to a std::regex string
 *
 * Currently only supports decimal and width specifiers such as \c \%d or \c
 * \%04d.
 *
 * @ingroup Util
 */
std::string FormatStrToRegexStr(const std::string& s);
}  // namespace volcart
