#pragma once

/** @file */

#include <stdexcept>
#include <string>

namespace volcart
{

/**
 * @class IOException
 * @brief IO operation exception
 *
 * Should be thrown when encountering errors reading or writing data.
 *
 * Based on
 * <a href="http://stackoverflow.com/a/8152888">this implementation</a>.
 *
 * @ingroup Types
 */
class IOException : public std::runtime_error
{
public:
    /**@{*/
    /** Constructor */
    explicit IOException(const char* msg) : std::runtime_error(msg) {}

    /** @copydoc IOException(const char* msg) */
    explicit IOException(const std::string& msg) : std::runtime_error(msg) {}
    /**@}*/
};
}  // namespace volcart
