#include <iostream>
#include <regex>

#include <gtest/gtest.h>

#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Logging.hpp"

namespace vc = volcart;
namespace vcl = volcart::logging;

// Setup matching patterns
std::string RGX_DATE = "[0-9]{4}-[0-9]{2}-[0-9]{2}";
std::string RGX_TIME = "[0-9]{2}:[0-9]{2}:[0-9]{2}\\.[0-9]{3}";
std::string RGX_NMSP = "\\[volcart\\]";
std::string RGX_PRFX = "\\[" + RGX_DATE + " " + RGX_TIME + "\\] " + RGX_NMSP;
std::string RGX_INFO = "\\[info\\]";
std::string RGX_WARN = "\\[warning\\]";
std::string RGX_ERRO = "\\[error\\]";

TEST(Logging, LoggerIsInitialized)
{
    // Retrieval
    EXPECT_NE(volcart::Logger(), nullptr);
}

TEST(Logging, AddLogFile)
{
    //// Setup the log file ////
    auto logPath = "vc_core_LoggingTest_" + vc::DateTime() + ".log";
    ASSERT_NO_THROW(vcl::AddLogFile(logPath));

    //// Write test messages ////
    auto infoMsg = "Info message";
    vc::Logger()->info(infoMsg);

    auto warnMsg = "Warning message";
    vc::Logger()->warn(warnMsg);

    auto errorMsg = "Error message";
    vc::Logger()->error(errorMsg);

    //// Flush the file logger to disk ////
    vc::Logger()->flush();

    //// Check that the log file got created and can be opened ////
    std::ifstream logFile(logPath);
    ASSERT_TRUE(logFile.is_open());

    //// Check the messages ////
    std::string line;

    // Info message
    std::getline(logFile, line);
    std::regex rgxInfo(RGX_PRFX + " " + RGX_INFO + " " + infoMsg);
    EXPECT_TRUE(std::regex_match(line, rgxInfo));

    // Warning message
    std::getline(logFile, line);
    std::regex rgxWarn(RGX_PRFX + " " + RGX_WARN + " " + warnMsg);
    EXPECT_TRUE(std::regex_match(line, rgxWarn));

    // Warning message
    std::getline(logFile, line);
    std::regex rgxError(RGX_PRFX + " " + RGX_ERRO + " " + errorMsg);
    EXPECT_TRUE(std::regex_match(line, rgxError));

    logFile.close();
}
