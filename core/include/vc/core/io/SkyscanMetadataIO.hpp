#pragma once

/** @file */

#include <regex>

#include <boost/filesystem.hpp>

#include "vc/core/types/Metadata.hpp"

namespace volcart
{
/**
 * @class SkyscanMetadataIO
 * @author Tam Nguyen
 * @date 10/16/18
 *
 * @brief Read a Skyscan metadata log
 *
 * Parses and saves necessary data from slice image directory log file, such as
 * slice image prefix
 *
 * @ingroup IO
 */
class SkyscanMetadataIO
{
public:
    /** @brief Set the log file path */
    void setPath(const boost::filesystem::path& p) { path_ = p; }

    /** @brief Read log file and return metadata object */
    Metadata read();

    std::string getSliceRegexString();

private:
    /** @brief parse the file */
    void parse_();

    /** Path to the log file */
    boost::filesystem::path path_;

    /** Metadata object storing log file values */
    Metadata metadata_;
};

}  // namespace volcart