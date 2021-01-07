#include "vc/core/util/FormatStrToRegexStr.hpp"

#include <regex>

std::string volcart::FormatStrToRegexStr(const std::string& s)
{
    std::string result;

    // Sanitize the string
    // From: https://stackoverflow.com/a/40195721
    std::regex special{R"([[\^$.|?*+(){}])"};
    result = std::regex_replace(s, special, R"(\$&)");

    // Replace the decimal values
    std::regex decim{"%([0-9]*)d"};
    result = std::regex_replace(result, decim, "([[:digit:]]{$01})");

    // Replace empty braces with *
    // This is what I expect, but might not be what others expect
    std::regex emptyBrace{"\\{\\}"};
    result = std::regex_replace(result, emptyBrace, "*");

    return result;
}