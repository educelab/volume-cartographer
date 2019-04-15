/**
 * @file Logging.hpp
 *
 * @ingroup Util
 */

#pragma once

#include <boost/filesystem.hpp>

#include "spdlog/sinks/dist_sink.h"
#include "spdlog/spdlog.h"

/**
 * @namespace volcart::logging
 * @brief Logging utilities
 */
namespace volcart
{
namespace logging
{
/** Logger type */
using Logger = std::shared_ptr<spdlog::logger>;

/**
 * Global list of all logger sinks
 *
 * @warning Initialized by volcart::logging::Init(). Modify with care.
 */
static std::shared_ptr<spdlog::sinks::dist_sink_mt> loggers;

/** Logging system initialization */
Logger Init();

/**
 * @brief Add a log file output to the logger
 *
 * @throw spdlog::spdlog_ex
 */
void AddLogFile(const boost::filesystem::path& path);
}  // namespace logging

/** @brief Volume Cartographer global logger */
static logging::Logger logger = logging::Init();
}  // namespace volcart
