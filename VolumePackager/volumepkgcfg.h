#include <iostream>
#include <fstream>

#include "picojson.h"

class VolumePkgCfg {
public:
	VolumePkgCfg(std::string);
	VolumePkgCfg(picojson::value);
	int getInt(std::string);
	double getDouble(std::string);
	std::string getString(std::string, std::string);
	VolumePkgCfg getInnerElement(std::string);
private:
	picojson::value json_data;
};