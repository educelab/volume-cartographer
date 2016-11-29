// VC Metadata
// Generic interface for storing metadata as a JSON payload.
// Created by Seth Parker on 10/27/15.

// Can (and perhaps should) be extended to support more specific functionality.
// See "volumepackage/volumepkgcfg.h" as an example.
#pragma once

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

#include "external/json.hpp"

#include "core/vc_defines.h"

namespace volcart
{
class Metadata
{

public:
    Metadata() {}

    Metadata(boost::filesystem::path file_location);

    // Path
    boost::filesystem::path path() const { return _path; };
    void setPath(const std::string& path) { _path = path; };

    // Save to file
    void save(const boost::filesystem::path& path);
    void save() { save(_path); };

    // Debug function
    void printString() const { std::cout << _json << std::endl; }
    void printObject() const { std::cout << _json.dump(4) << std::endl; }

    // Retrieval
    template <typename T>
    T get(const std::string& key) const
    {
        if (_json.find(key) == _json.end()) {
            auto msg = "could not find key '" + key + "' in metadata";
            throw std::runtime_error(msg);
        }
        return _json[key].get<T>();
    }

    // Assignment
    template <typename T>
    void set(const std::string& key, T value)
    {
        _json[key] = value;
    }

protected:
    nlohmann::json _json;
    boost::filesystem::path _path;
};
}  //  namespace volcart
