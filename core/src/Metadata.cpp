#include "vc/core/types/Metadata.hpp"

#include <fstream>
#include <iostream>

#include "vc/core/types/Exceptions.hpp"

namespace fs = volcart::filesystem;

using namespace volcart;

// Read a json config from disk
Metadata::Metadata(const fs::path& fileLocation) : path_{fileLocation}
{
    // open the file
    if (not fs::exists(fileLocation)) {
        auto msg = "could not find json file '" + fileLocation.string() + "'";
        throw IOException(msg);
    }
    std::ifstream jsonFile(fileLocation.string());
    if (not jsonFile) {
        auto msg = "could not open json file '" + fileLocation.string() + "'";
        throw IOException(msg);
    }

    jsonFile >> json_;
    if (jsonFile.bad()) {
        auto msg = "could not read json file '" + fileLocation.string() + "'";
        throw IOException(msg);
    }
}

auto Metadata::path() const -> filesystem::path { return path_; }

void Metadata::setPath(const filesystem::path& path) { path_ = path; }

void Metadata::save() const { save(path_); }

// save the JSON file to disk
void Metadata::save(const fs::path& path) const
{
    // open the file
    std::ofstream jsonFile(path.string(), std::ofstream::out);

    // try to push into the json file
    jsonFile << json_ << '\n';
    if (jsonFile.fail()) {
        const auto msg = "could not write json file '" + path.string() + "'";
        throw IOException(msg);
    }
}

auto Metadata::hasKey(const std::string& key) const -> bool
{
    return json_.contains(key);
}

void Metadata::printObject() const { std::cout << json_.dump(4) << std::endl; }
