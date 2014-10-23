#include "volumepkgcfg.h"

VolumePkgCfg::VolumePkgCfg(std::string file_location) {
	
	// open the file
	std::ifstream json_file(file_location);
	if (!json_file.is_open()) {
		std::cout << "Json File " << file_location << 
			" not found" << std::endl;
	}

	// try to push into a picojson value
	json_file >> json_data;
	if (json_file.fail()) {
		std::cerr << picojson::get_last_error() << std::endl;
	}
}

VolumePkgCfg::VolumePkgCfg(picojson::value val) {
	json_data = val;
}

int VolumePkgCfg::getInt(std::string identifier) {
	return (int) json_data.get(identifier).get<double>();
}
double VolumePkgCfg::getDouble(std::string identifier) {
	return json_data.get(identifier).get<double>();
}
std::string VolumePkgCfg::getString(std::string identifier, std::string default_output = "") {
	try {
		return json_data.get(identifier).get<std::string>();
	}
	catch(...) {
		return default_output;
	}
}

VolumePkgCfg VolumePkgCfg::getInnerElement(std::string identifier) {
	return VolumePkgCfg(json_data.get(identifier));
}