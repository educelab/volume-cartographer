// VC Metadata
// Generic interface for storing metadata as a JSON payload.
// Created by Seth Parker on 10/27/15.

// Can (and perhaps should) be extended to support more specific functionality.
// See "volumepackage/volumepkgcfg.h" as an example.

#ifndef _VC_METADATA_H_
#define _VC_METADATA_H_

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "common/picojson.h"

#include "common/vc_defines.h"

namespace volcart
{
class Metadata
{

public:
    Metadata();
    Metadata(const boost::filesystem::path& file_location);

    // Path
    boost::filesystem::path path() const { return _path; };
    void setPath(const std::string& path) { _path = path; };

    // Save to file
    void save(const boost::filesystem::path& path);
    void save() { save(_path.string()); };

    // Debug functions
    void printString() const;
    void printObject() const;

    // Retrieval
    int getInt(const std::string& key) const;
    double getDouble(const std::string& key) const;
    std::string getString(const std::string& key) const;

    // Assignment
    void setValue(const std::string& key, int value);
    void setValue(const std::string& key, unsigned long value);
    void setValue(const std::string& key, double value);
    void setValue(const std::string& key, const std::string& value);

protected:
    picojson::value _json;
    boost::filesystem::path _path;
};
}

#endif  // _VC_METADATA_H_
