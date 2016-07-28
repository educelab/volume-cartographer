#include "volumepkgcfg.h"

///// CONSTRUCTORS /////
VolumePkgCfg::VolumePkgCfg() { _json["version"] = VOLPKG_VERSION; }

// Construct a new json config using a Dictionary as a template
VolumePkgCfg::VolumePkgCfg(const volcart::Dictionary& dict, double version)
{
    // Populate the cfg with keys from the dict
    for (const auto& entry : dict) {
        if (entry.first == "version") {
            _json["version"] = version;
            continue;
        }

        // Default values
        if (entry.second == "int") {
            _json[entry.first] = 0;
        } else if (entry.second == "double") {
            _json[entry.first] = 0.0;
        } else {
            _json[entry.first] = "";
        }
    }
};
