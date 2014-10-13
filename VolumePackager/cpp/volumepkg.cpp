#include "volumepkg.h"

VolumePkg::VolumePkg(std::string file_location) : config(file_location){
	VolumePkgCfg config = VolumePkgCfg(file_location + "config.json");
}

int VolumePkg::getNumberOfSlices() {
	return config.getInt("number of slices");
}


int main() {
	VolumePkg vpkg = VolumePkg("test");
}