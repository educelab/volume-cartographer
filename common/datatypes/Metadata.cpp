#include "Metadata.h"

namespace volcart {

    ///// CONSTRUCTORS /////
    Metadata::Metadata() {
        picojson::object obj;
        _json = picojson::value(obj);
    }

    // Read a json config from disk
    Metadata::Metadata(std::string file_location) {

        _path = file_location;

        // open the file
        std::ifstream json_file(file_location);
        if (!json_file.is_open()) {
            std::cout << "Json File " << file_location <<
            " not found" << std::endl;
        }

        // try to push into a picojson value
        json_file >> _json;
        if (json_file.fail()) {
            std::cerr << picojson::get_last_error() << std::endl;
        }

    }


// save the JSON file to disk
    void Metadata::save(std::string path) {

        // open the file
        std::ofstream json_file (path, std::ofstream::out);

        // try to push into the json file
        json_file << _json << std::endl;
        if (json_file.fail()) {
            std::cerr << picojson::get_last_error() << std::endl;
        }

        json_file.close();
    }


// debug
    void Metadata::printString() {
        std::cout << _json << std::endl;
    }

    void Metadata::printObject() {
        for (picojson::object::const_iterator i = _json.get<picojson::object>().begin(); i != _json.get<picojson::object>().end(); ++i) {
            std::cout << i->first << "  " << i->second << std::endl;
        }
    }


// retrieval
    int Metadata::getInt(std::string identifier) {
        if ( _json.get(identifier).is<double>() )
            return (int) _json.get(identifier).get<double>();
        else
            return 0;
    }

    double Metadata::getDouble(std::string identifier) {
        if ( _json.get(identifier).is<double>() )
            return _json.get(identifier).get<double>();
        else
            return 0.0;
    }

    std::string Metadata::getString(std::string identifier) {
        try {
            return _json.get(identifier).get<std::string>();
        }
        catch(...) {
            return "NULL";
        }
    }


// assignment
    void Metadata::setValue(std::string identifier, int value) {
        // picojson requires int be cast to double
        picojson::object jsonObject = _json.get<picojson::object>();
        jsonObject[identifier] = picojson::value(double(value));
        _json = picojson::value(jsonObject);
    }

    void Metadata::setValue(std::string identifier, unsigned long value) {
        picojson::object jsonObject = _json.get<picojson::object>();
        jsonObject[identifier] = picojson::value(double(value));
        _json = picojson::value(jsonObject);
    }

    void Metadata::setValue(std::string identifier, double value) {
        picojson::object jsonObject = _json.get<picojson::object>();
        jsonObject[identifier] = picojson::value(value);
        _json = picojson::value(jsonObject);
    }

    void Metadata::setValue(std::string identifier, std::string value) {
        picojson::object jsonObject = _json.get<picojson::object>();
        jsonObject[identifier] = picojson::value(value);
        _json = picojson::value(jsonObject);

    }

} // namespace volcart