#ifndef _VOLUMEPKGCFG_H_
#define _VOLUMEPKGCFG_H_

#include <iostream>
#include <fstream>

#include "picojson.h"

class VolumePkgCfg {
public:
    VolumePkgCfg(std::string);
    VolumePkgCfg(picojson::value);
    void saveCfg(std::string);

    // Debug functions
    void printString();
    void printObject();

    // Retrieval
    int getInt(std::string);
    double getDouble(std::string);
    std::string getString(std::string, std::string);
    VolumePkgCfg getInnerElement(std::string);

    // Assignment
    void setValue(std::string, int);
    void setValue(std::string, double);
    void setValue(std::string, std::string);

private:
    picojson::value jsonString;
    picojson::object jsonObject;
};

#endif // _VOLUMEPKGCFG_H_
