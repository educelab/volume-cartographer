#pragma once

#include <stdexcept>
#include <string>

namespace volcart
{

// From: http://stackoverflow.com/a/8152888
class IOException : public std::exception
{
public:
    explicit IOException(const char* msg) : msg_(msg) {}
    explicit IOException(std::string msg) : msg_(std::move(msg)) {}
    const char* what() const noexcept override { return msg_.c_str(); }

protected:
    std::string msg_;
};
}  // namespace volcart
