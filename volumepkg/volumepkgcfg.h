#ifndef _VOLUMEPKGCFG_H_
#define _VOLUMEPKGCFG_H_

#include <iostream>
#include <fstream>

#include "picojson.h"

#include "vc_defines.h"
#include "volumepkg_version.h"

class VolumePkgCfg {
public:
    VolumePkgCfg();
    VolumePkgCfg(std::string);
    VolumePkgCfg(volcart::Dictionary dict, double version = VOLPKG_VERSION);

    void saveCfg(std::string);

    // Debug functions
    void printString();
    void printObject();

    // Retrieval
    int getInt(std::string);
    double getDouble(std::string);
    std::string getString(std::string, std::string);
    //VolumePkgCfg getInnerElement(std::string);

    // Assignment
    void setValue(std::string, int);
    void setValue(std::string, double);
    void setValue(std::string, std::string);

private:
    picojson::value _json;
};

#endif // _VOLUMEPKGCFG_H_
