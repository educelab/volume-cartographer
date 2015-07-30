#include "volumepkgcfg.h"

VolumePkgCfg::VolumePkgCfg() {
    picojson::object obj;
    obj["version"] = picojson::value(VOLPKG_VERSION);
    _json = picojson::value(obj);
}

VolumePkgCfg::VolumePkgCfg(VolumePkgCfg& cfg) {
    this->_json = cfg._json;
}

VolumePkgCfg::VolumePkgCfg(std::string file_location) {

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

VolumePkgCfg::VolumePkgCfg(picojson::value val) {
    if (val.is<picojson::object>()) {
        _json = val;
    } else {
        std::cout << "input is not a json object" << std::endl;
    }
};

VolumePkgCfg::VolumePkgCfg(picojson::object obj) {
    _json = picojson::value(obj);
};

VolumePkgCfg::VolumePkgCfg(volcart::Dictionary dict, double version) {

    // Populate the cfg with keys from the dict
    picojson::object obj;
    volcart::Dictionary::const_iterator entry = dict.begin();
    while ( entry != dict.end() ) {

        // Version must be populated correctly for volumepkg setMetadata() functions to work
        if ( entry->first == "version" ) {
            obj["version"] = picojson::value(version);

            ++entry;
            continue;
        }

        // Default values
        std::string type = entry->second;
        if ( type == "int" ) {
            int initValue = 0;
            obj[entry->first] = picojson::value(double(initValue));
        }
        else
        if ( type == "double" ) {
            double initValue = 0.0;
            obj[entry->first] = picojson::value(initValue);
        }
        else {
            std::string initValue = "";
            obj[entry->first] = picojson::value(initValue);
        }

        ++entry;
    }

    _json = picojson::value(obj);
};

void VolumePkgCfg::saveCfg(std::string file_location) {
    
    // open the file
    std::ofstream json_file (file_location, std::ofstream::out);

    // try to push into the json file
    json_file << _json << std::endl;
    if (json_file.fail()) {
        std::cerr << picojson::get_last_error() << std::endl;
    }

    json_file.close();
}


// debug
void VolumePkgCfg::printString() {
    std::cout << _json << std::endl;
}

void VolumePkgCfg::printObject() {
    for (picojson::object::const_iterator i = _json.get<picojson::object>().begin(); i != _json.get<picojson::object>().end(); ++i) {
        std::cout << i->first << "  " << i->second << std::endl;
    }
}


// retrieval
int VolumePkgCfg::getInt(std::string identifier) {
    if ( _json.get(identifier).is<double>() )
        return (int) _json.get(identifier).get<double>();
    else
        return NULL;
}

double VolumePkgCfg::getDouble(std::string identifier) {
    if ( _json.get(identifier).is<double>() )
        return _json.get(identifier).get<double>();
    else
        return NULL;
}

std::string VolumePkgCfg::getString(std::string identifier, std::string default_output) {
    try {
        return _json.get(identifier).get<std::string>();
    }
    catch(...) {
        return default_output;
    }
}

// This is unused and needs to be rethought
//VolumePkgCfg VolumePkgCfg::getInnerElement(std::string identifier) {
//    return VolumePkgCfg(_json.get(identifier));
//}


// assignment
void VolumePkgCfg::setValue(std::string identifier, int value) {
    // picojson requires int be cast to double
    picojson::object jsonObject = _json.get<picojson::object>();
    jsonObject[identifier] = picojson::value(double(value));
    _json = picojson::value(jsonObject);
}

void VolumePkgCfg::setValue(std::string identifier, double value) {
    picojson::object jsonObject = _json.get<picojson::object>();
    jsonObject[identifier] = picojson::value(value);
    _json = picojson::value(jsonObject);
}

void VolumePkgCfg::setValue(std::string identifier, std::string value) {
    picojson::object jsonObject = _json.get<picojson::object>();
    jsonObject[identifier] = picojson::value(value);
    _json = picojson::value(jsonObject);
}