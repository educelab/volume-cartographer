#define BOOST_TEST_MODULE Logging

#include <iostream>
#include <regex>

#include <boost/test/unit_test.hpp>

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

BOOST_AUTO_TEST_CASE(LoggerIsInitialized)
{
    // Constructor
    vcl::Init();

    // Retrieval
    BOOST_TEST(volcart::logger != nullptr);
}

BOOST_AUTO_TEST_CASE(AddLogFile)
{
    //// Setup the log file ////
    auto logPath = "vc_core_LoggingTest_" + vc::DateTime() + ".log";
    vcl::AddLogFile(logPath);
    std::cout << "Writing to log file: " << logPath << std::endl;

    //// Write test messages ////
    auto infoMsg = "Info message";
    vc::logger->info(infoMsg);

    auto warnMsg = "Warning message";
    vc::logger->warn(warnMsg);

    auto errorMsg = "Error message";
    vc::logger->error(errorMsg);

    //// Flush the file logger to disk ////
    vc::logger->flush();

    //// Check that the log file got created and can be opened ////
    std::ifstream logFile(logPath);
    BOOST_TEST_REQUIRE(logFile.is_open());

    //// Check the messages ////
    std::string line;

    // Info message
    std::getline(logFile, line);
    std::regex rgxInfo(RGX_PRFX + " " + RGX_INFO + " " + infoMsg);
    BOOST_CHECK(std::regex_match(line, rgxInfo));

    // Warning message
    std::getline(logFile, line);
    std::regex rgxWarn(RGX_PRFX + " " + RGX_WARN + " " + warnMsg);
    BOOST_CHECK(std::regex_match(line, rgxWarn));

    // Warning message
    std::getline(logFile, line);
    std::regex rgxError(RGX_PRFX + " " + RGX_ERRO + " " + errorMsg);
    BOOST_CHECK(std::regex_match(line, rgxError));

    logFile.close();
}
