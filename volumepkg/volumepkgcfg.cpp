#include "volumepkgcfg.h"

///// CONSTRUCTORS /////
// Construct a new json config using a Dictionary as a template
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