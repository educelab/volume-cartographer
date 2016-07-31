#include "types/Metadata.h"

namespace fs = boost::filesystem;

namespace volcart
{

///// CONSTRUCTORS /////
Metadata::Metadata() : _json(picojson::value(picojson::object())) {}

// Read a json config from disk
Metadata::Metadata(const fs::path& file_location) : _path(file_location)
{
    // open the file
    std::ifstream json_file(file_location.string());
    if (!json_file.is_open()) {
        std::cout << "Json File " << file_location << " not found" << std::endl;
    }

    // try to push into a picojson value
    json_file >> _json;
    if (json_file.fail()) {
        std::cerr << picojson::get_last_error() << std::endl;
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
        std::cerr << picojson::get_last_error() << std::endl;
    }

    json_file.close();
}

// debug
void Metadata::printString() const { std::cout << _json << std::endl; }

void Metadata::printObject() const
{
    for (const auto& obj : _json.get<picojson::object>()) {
        std::cout << obj.first << " " << obj.second << std::endl;
    }
}

// retrieval
int Metadata::getInt(const std::string& key) const
{
    auto val = _json.get(key);
    return (val.is<double>() ? val.get<double>() : 0);
}

double Metadata::getDouble(const std::string& key) const
{
    auto val = _json.get(key);
    return (val.is<double>() ? val.get<double>() : 0.0);
}

std::string Metadata::getString(const std::string& key) const
{
    try {
        return _json.get(key).get<std::string>();
    } catch (const std::runtime_error& err) {
        return "NULL";
    }
}

// assignment
void Metadata::setValue(const std::string& key, int value)
{
    // picojson requires int be cast to double
    picojson::object jsonObject = _json.get<picojson::object>();
    jsonObject[key] = picojson::value(double(value));
    _json = picojson::value(jsonObject);
}

void Metadata::setValue(const std::string& key, unsigned long value)
{
    picojson::object jsonObject = _json.get<picojson::object>();
    jsonObject[key] = picojson::value(double(value));
    _json = picojson::value(jsonObject);
}

void Metadata::setValue(const std::string& key, double value)
{
    picojson::object jsonObject = _json.get<picojson::object>();
    jsonObject[key] = picojson::value(value);
    _json = picojson::value(jsonObject);
}

void Metadata::setValue(const std::string& key, const std::string& value)
{
    picojson::object jsonObject = _json.get<picojson::object>();
    jsonObject[key] = picojson::value(value);
    _json = picojson::value(jsonObject);
}
}  // namespace volcart
