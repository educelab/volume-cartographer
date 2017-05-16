/**
 * @file DateTime.hpp
 *
 * @ingroup Core
 */
#pragma once

#include <array>
#include <ctime>
#include <iostream>

namespace volcart
{
/** @brief Returns a string representation of the current date and time
 *
 * Format: YYYYMMDDHMS
 */
inline std::string DateTime()
{
    std::array<char, 24> buf{};
    auto now = std::time(nullptr);
    std::strftime(buf.data(), buf.size(), "%Y%m%d%H%M%S", std::localtime(&now));
    return std::string(buf.data());
}

}