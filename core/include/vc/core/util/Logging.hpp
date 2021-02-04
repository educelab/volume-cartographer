#pragma once

/**
 * @file Logging.hpp
 *
 * @ingroup Util
 */

#include <boost/filesystem.hpp>
#include <spdlog/spdlog.h>

/**
 * @namespace volcart::logging
 * @brief Logging utilities
 */
namespace volcart
{
namespace logging
{
/**
 * @brief Add a log file output to the logger
 *
 * @throw spdlog::spdlog_ex
 */
void AddLogFile(const boost::filesystem::path& path);

/** @brief Set the logging level */
void SetLogLevel(const std::string& s);
}  // namespace logging

/** @brief Volume Cartographer global logger */
auto Logger() -> std::shared_ptr<spdlog::logger>;

}  // namespace volcart
