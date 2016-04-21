#include "volumepkgcfg.h"

///// CONSTRUCTORS /////
VolumePkgCfg::VolumePkgCfg()
{
    picojson::object obj;
    obj["version"] = picojson::value(VOLPKG_VERSION);
    _json = picojson::value(obj);
}

// Construct a new json config using a Dictionary as a template
VolumePkgCfg::VolumePkgCfg(const volcart::Dictionary& dict, double version)
{
    // Populate the cfg with keys from the dict
    picojson::object obj;
    volcart::Dictionary::const_iterator entry = dict.begin();
    for (const auto& entry : dict) {
        if (entry.first == "version") {
            obj["version"] = picojson::value(version);
            continue;
        }

        // Default values
        if (entry.second == "int") {
            int initValue = 0;
            obj[entry.first] = picojson::value(double(initValue));
        } else if (entry.second == "double") {
            double initValue = 0;
            obj[entry.first] = picojson::value(initValue);
        } else {
            std::string initValue{""};
            obj[entry.first] = picojson::value(initValue);
        }
    }
    _json = picojson::value(obj);
};
