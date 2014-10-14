#include "volumepkg.h"

VolumePkg::VolumePkg(std::string file_location) : config(file_location + "/config.json"){
	location = file_location;
}

int VolumePkg::getNumberOfSlices() {
	return config.getInt("number of slices");
}

int VolumePkg::getNumberOfSliceCharacters() {
	int num_slices = getNumberOfSlices();
	int num_characters = 0;
	while (num_slices > 0) {
		num_characters += 1;
		num_slices /= 10;
	}
	return num_characters;
}

cv::Mat VolumePkg::getSliceAtIndex(int index) {

	//get the file name
	std::string slice_location(location);
	slice_location += config.getString("slice location", "/slices/");
	int num_slice_characters = getNumberOfSliceCharacters();
	std::string str_index = std::to_string(index);
	int num_leading_zeroes = num_slice_characters - str_index.length();
	for (int i = 0; i < num_leading_zeroes; i++) {slice_location += '0';}
	slice_location += str_index;
	slice_location += ".tif";

	std::cout << "Location: " << slice_location << std::endl;
	//load the mat and return it
	return cv::imread(slice_location);
}


int main() {
	VolumePkg vpkg = VolumePkg("/Users/david/Desktop//volumepkg/");
	std::cout << "Number of Slices: " << vpkg.getNumberOfSlices() << std::endl;
	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", vpkg.getSliceAtIndex(1) );
    cv::waitKey(0); 
}