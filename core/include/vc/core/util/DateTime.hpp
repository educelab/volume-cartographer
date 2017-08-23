/**
 * @file DateTime.hpp
 *
 * @ingroup Core
 */
#pragma once

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace volcart
{
/** @brief Returns a string representation of the current date and time
 *
 * Format: YYYYMMDDHMS
 *
 * Code from <a href="http://stackoverflow.com/a/16358264/1917043">here</a>.
 */
inline std::string DateTime()
{
    auto now = std::time(nullptr);
    auto time = std::localtime(&now);

    std::stringstream ss;
    ss << std::put_time(time, "%Y%m%d%H%M%S");
    return ss.str();
}
}