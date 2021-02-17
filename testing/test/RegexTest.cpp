#include <regex>

#include <gtest/gtest.h>

#include "vc/core/filesystem.hpp"

namespace fs = volcart::filesystem;

/*
 * Tests for std::regex
 * */
class RegexFixture : public ::testing::Test
{
public:
    RegexFixture() = default;
};

/*
 * Test goal: verify that the given regex successfully parses a filename
 * that consists of leading non-numeric characters and trailing numeric
 * characters.
 * */
TEST_F(RegexFixture, ParseMixedFilenameWithEndDigits)
{
    // Load test file directory
    fs::path inputPath = "fakepointset1234.vcps";

    // Match the regex
    static const std::regex SLICE_REG{"^.*?(\\d+)$"};
    std::smatch matches;

    // A file that doesn't match the regex should be ignored
    if (not std::regex_match(inputPath.stem().string(), matches, SLICE_REG)) {
        // This test vcps file has a valid name, so this is a failure case.
        FAIL();
    } else {
        // Confirm that the second match can be converted to an integer
        // First match is the whole regex
        // Second match should be the capture group of one or more digits: \\d+
        ASSERT_NO_FATAL_FAILURE(std::stoi(matches[1]));
    }
}

/*
 * Test goal: verify that the given regex will not match if
 * the filename contains no ending numeric characters, but numeric
 * characters are between non-numeric characters.
 * */
TEST_F(RegexFixture, ParseFilenameWithoutEndDigits)
{
    // Load test file directory
    fs::path inputPath = "fake1234pointset.vcps";

    // Match the regex
    static const std::regex SLICE_REG{"^.*?(\\d+)$"};
    std::smatch matches;

    // A file that doesn't match the regex should be ignored
    if (std::regex_match(inputPath.stem().string(), matches, SLICE_REG)) {
        // The regex should not match this file name.
        FAIL();
    }
}

/*
 * Test goal: verify that the given regex will not match if
 * the filename contains only non-numeric characters.
 * */
TEST_F(RegexFixture, ParseNonNumericFilename)
{
    // Load test file directory
    fs::path inputPath = "fakepointset.vcps";

    // Match the regex
    static const std::regex SLICE_REG{"^.*?(\\d+)$"};
    std::smatch matches;

    // A file that doesn't match the regex should be ignored
    if (std::regex_match(inputPath.stem().string(), matches, SLICE_REG)) {
        // The regex should not match this file name.
        FAIL();
    }
}