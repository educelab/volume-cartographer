/**
 * @file DateTime.hpp
 *
 * @ingroup Core
 */
#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace volcart
{
/**
 * @brief Returns a string representation of the current date and time
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

/**
 * @brief Returns a string representation of the provided time duration
 *
 * Format: dd:hh:mm:ss
 *
 * Code from <a href="https://stackoverflow.com/a/46134506">here</a>.
 */
template <typename Rep, typename Period>
std::string DurationToDurationString(std::chrono::duration<Rep, Period> input)
{
    using namespace std::chrono;
    using days = duration<int, std::ratio<86400>>;
    auto d = duration_cast<days>(input);
    input -= d;
    auto h = duration_cast<hours>(input);
    input -= h;
    auto m = duration_cast<minutes>(input);
    input -= m;
    auto s = duration_cast<seconds>(input);

    auto dc = d.count();
    auto hc = h.count();
    auto mc = m.count();
    auto sc = s.count();

    std::stringstream ss;
    ss.fill('0');
    if (dc) {
        ss << d.count() << "d";
    }
    if (dc || hc) {
        if (dc) {
            ss << std::setw(2);
        }  // pad if second set of numbers
        ss << h.count() << "h";
    }
    if (dc || hc || mc) {
        if (dc || hc) {
            ss << std::setw(2);
        }
        ss << m.count() << "m";
    }
    if (dc || hc || mc || sc) {
        if (dc || hc || mc) {
            ss << std::setw(2);
        }
        ss << s.count() << 's';
    }

    return ss.str();
}
}  // namespace volcart
