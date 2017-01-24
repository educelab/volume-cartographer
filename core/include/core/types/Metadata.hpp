// VC Metadata
// Generic interface for storing metadata as a JSON payload.
// Created by Seth Parker on 10/27/15.

// Can (and perhaps should) be extended to support more specific functionality.
// See "volumepackage/volumepkgcfg.hpp" as an example.
#pragma once

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>

#include "core/vc_defines.hpp"
#include "external/json.hpp"

namespace volcart
{
class Metadata
{

public:
    Metadata() {}
    explicit Metadata(boost::filesystem::path fileLocation);

    // Path
    boost::filesystem::path path() const { return path_; }
    void setPath(const std::string& path) { path_ = path; }

    // Save to file
    void save(const boost::filesystem::path& path);
    void save() { save(path_); }

    // Debug function
    void printString() const { std::cout << json_ << std::endl; }
    void printObject() const { std::cout << json_.dump(4) << std::endl; }

    // Retrieval
    template <typename T>
    T get(const std::string& key) const
    {
        if (json_.find(key) == json_.end()) {
            auto msg = "could not find key '" + key + "' in metadata";
            throw std::runtime_error(msg);
        }
        return json_[key].get<T>();
    }

    // Assignment
    template <typename T>
    void set(const std::string& key, T value)
    {
        json_[key] = value;
    }

protected:
    nlohmann::json json_;
    boost::filesystem::path path_;
};
}
