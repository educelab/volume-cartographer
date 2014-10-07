#include <iostream>
#include <fstream>

#include "picojson.h"

class VolumePkg {
public:
	VolumePkg(std::string);
	VolumePkg(picojson::value);
	int getInt(std::string);
	double getDouble(std::string);
	std::string getString(std::string);
	VolumePkg getInnerElement(std::string);
private:
	picojson::value json_data;
};