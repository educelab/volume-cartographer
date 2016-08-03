#include "Metadata.h"

namespace fs = boost::filesystem;

namespace volcart
{

// Read a json config from disk
Metadata::Metadata(const fs::path& file_location) : _path(file_location)
{
    // open the file
    std::ifstream json_file(file_location.string());
    if (!json_file.is_open()) {
        std::cout << "Json File " << file_location << " not found" << std::endl;
        throw std::ifstream::failure("could not find json file");
    }

    json_file >> _json;
    if (json_file.fail()) {
        std::cerr << "could not read json file " << file_location << std::endl;
        throw std::ifstream::failure("could not read json file");
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
        std::cerr << "could not save json file " << path << std::endl;
    }
}

}  // namespace volcart
