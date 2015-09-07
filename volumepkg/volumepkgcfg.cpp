#include "volumepkgcfg.h"

VolumePkgCfg::VolumePkgCfg(std::string file_location){

    // open the file
    std::ifstream json_file(file_location);
    if (!json_file.is_open()) {
        std::cout << "Json File " << file_location <<
            " not found" << std::endl;
    }

    // try to push into a picojson value
    json_file >> jsonString;
    if (json_file.fail()) {
        std::cerr << picojson::get_last_error() << std::endl;
    }

    if (jsonString.is<picojson::object>()) {
        jsonObject = jsonString.get<picojson::object>();
    } else {
        std::cout << "input is not a json object" << std::endl;
    }
}

VolumePkgCfg::VolumePkgCfg(picojson::value val) {

    if (val.is<picojson::object>()) {
        jsonObject = val.get<picojson::object>();
    } else {
        std::cout << "input is not a json object" << std::endl;
    }
}

void VolumePkgCfg::saveCfg(std::string file_location) {
    
    // open the file
    std::ofstream json_file (file_location, std::ofstream::out);

    // try to push into the json file
    json_file << jsonString << std::endl;
    if (json_file.fail()) {
        std::cerr << picojson::get_last_error() << std::endl;
    }

    json_file.close();
}


// debug
void VolumePkgCfg::printString() {
    std::cout << jsonString << std::endl;
}

void VolumePkgCfg::printObject() {
    for (picojson::object::const_iterator i = jsonObject.begin(); i != jsonObject.end(); ++i) {
        std::cout << i->first << "  " << i->second << std::endl;
    }
}


// retrieval
int VolumePkgCfg::getInt(std::string identifier) {
    if ( jsonString.get(identifier).is<double>() )
        return (int) jsonString.get(identifier).get<double>();
    else
        return NULL;
}

double VolumePkgCfg::getDouble(std::string identifier) {
    if ( jsonString.get(identifier).is<double>() )
        return jsonString.get(identifier).get<double>();
    else
        return NULL;
}

std::string VolumePkgCfg::getString(std::string identifier, std::string default_output) {
    try {
        return jsonString.get(identifier).get<std::string>();
    }
    catch(...) {
        return default_output;
    }
}

VolumePkgCfg VolumePkgCfg::getInnerElement(std::string identifier) {
    return VolumePkgCfg(jsonString.get(identifier));
}


// assignment
void VolumePkgCfg::setValue(std::string identifier, int value) {
    // picojson requires int be cast to double
    jsonObject[identifier] = picojson::value(double(value));
    jsonString = picojson::value(jsonObject);
}

void VolumePkgCfg::setValue(std::string identifier, double value) {
    jsonObject[identifier] = picojson::value(value);
    jsonString = picojson::value(jsonObject);
}

void VolumePkgCfg::setValue(std::string identifier, std::string value) {
    jsonObject[identifier] = picojson::value(value);
    jsonString = picojson::value(jsonObject);
}