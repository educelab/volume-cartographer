#include "vc/core/util/Logging.hpp"

#include <iostream>
#include <memory>
#include <mutex>

#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/syslog_sink.h>

namespace fs = boost::filesystem;
namespace vcl = volcart::logging;

// SplitSink for printing to std::cout or std::cerr based on message level
// Adapted from: https://github.com/gabime/spdlog/wiki/4.-Sinks
template <typename Mutex>
class SplitSink : public spdlog::sinks::base_sink<Mutex>
{
protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        fmt::memory_buffer formatted;
        spdlog::sinks::sink::formatter_->format(msg, formatted);

        // Everything below error is printed to std::cout
        if (msg.level <= spdlog::level::warn) {
            std::cout << fmt::to_string(formatted);
        }

        // Everything error and above is printed to std::err
        else {
            std::cerr << fmt::to_string(formatted);
        }
    }

    // Flush all outputs
    void flush_() override
    {
        std::cout << std::flush;
        std::cerr << std::flush;
    }
};

// Thread safe SplitSink
using SplitSinkMT = SplitSink<std::mutex>;

vcl::Logger vcl::Init()
{
    // Lazy initialization of global loggers sink
    if (loggers == nullptr) {
        loggers = std::make_shared<spdlog::sinks::dist_sink_mt>();

        // Create default console logger
        auto consoleLogger = std::make_shared<SplitSinkMT>();
        loggers->add_sink(consoleLogger);
    }

    return std::make_shared<spdlog::logger>("volcart", loggers);
}

void vcl::AddLogFile(const fs::path& path)
{
    // Add a file logger to the logger list
    auto log =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string());
    loggers->add_sink(log);
}