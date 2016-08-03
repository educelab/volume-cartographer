#include "Metadata.h"

namespace fs = boost::filesystem;

namespace volcart
{

// Read a json config from disk
Metadata::Metadata(const fs::path& file_location) : _path(file_location)
{
    // open the file
    if (!fs::exists(file_location)) {
        auto msg = "could not find json file '" + file_location.string() + "'";
        throw std::runtime_error(msg);
    }
    std::ifstream json_file(file_location.string());
    if (!json_file) {
        auto msg = "could not open json file '" + file_location.string() + "'";
        throw std::ifstream::failure(msg);
    }

    json_file >> _json;
    if (json_file.bad()) {
        auto msg = "could not read json file '" + file_location.string() + "'";
        throw std::ifstream::failure(msg);
    }
}

// save the JSON file to disk
void Metadata::save(const fs::path& path)
{
    // open the file
    std::ofstream json_file(path.string(), std::ofstream::out);

    // try to push into the json file
    json_file << _json << std::endl;
    if (json_file.fail()) {
        auto msg = "could not write json file '" + path.string() + "'";
        throw std::ifstream::failure(msg);
    }
}

}  // namespace volcart
