#include "volumepkg.h"

VolumePkg::VolumePkg(std::string file_location, ) {
	
	// open the file
	std::ifstream json_file(file_location);
	if (!json_file.is_open()) {
		std::cout << "Json File " << file_location << 
			" not found" << std::endl;
		throw;
	}

	// try to push into a picojson value
	json_file >> json_data;
	if (json_file.fail()) {
		std::cerr << picojson::get_last_error() << std::endl;
		throw;
	}
}

VolumePkg::VolumePkg(picojson::value val) {
	json_data = val;
}

int VolumePkg::getInt(std::string identifier) {
	return (int) json_data.get(identifier).get<double>();
}
double VolumePkg::getDouble(std::string identifier) {
	return json_data.get(identifier).get<double>();
}
std::string VolumePkg::getString(std::string identifier) {
	return json_data.get(identifier).get<std::string>();
}

VolumePkg VolumePkg::getInnerElement(std::string identifier) {
	return VolumePkg(json_data.get(identifier));
}