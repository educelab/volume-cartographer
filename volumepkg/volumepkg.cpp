#include "volumepkg.h"

VolumePkg::VolumePkg(std::string file_location) : config(file_location + "/config.json"){
	location = file_location;
}

int VolumePkg::getNumberOfSlices() {
	return config.getInt("number of slices");
}

std::string VolumePkg::getPkgName() {
	return config.getString("volumepkg name", "UnnamedVolume");
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
	// REVISIT - Chao 20141104 - image format unified to 16UC1. An image is read as is,
	//           and converted to unsigned short grayscale (CV_16U) single channel.
	//           Without ANYCOLOR|ANYDEPTH, OpenCV will return 16UC1 TIFF as 8UC3.
	cv::Mat aSrcImg = cv::imread( slice_location, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
	cv::Mat aDstImg, aIntermediateImg;
	if ( aSrcImg.depth() == CV_8U ) {
		double minVal, maxVal;
		cv::minMaxLoc(aSrcImg, &minVal, &maxVal);
		aSrcImg.convertTo( aIntermediateImg, CV_16U, 65535.0/(maxVal - minVal), -minVal * 65535.0/(maxVal - minVal));
	} else {
		aSrcImg.copyTo( aIntermediateImg );
	}
	if ( aIntermediateImg.channels() > 1 ) {
		cv::cvtColor( aIntermediateImg, aDstImg, CV_BGR2GRAY ); // OpenCV use BGR to represent color image
	} else {
		aIntermediateImg.copyTo( aDstImg );
	}
	
	return aDstImg;
}

std::string VolumePkg::getNormalAtIndex(int index) {

	std::string ply_location(location);
	ply_location += config.getString("ply location", "/surface_normals/");

	int num_ply_chars = getNumberOfSliceCharacters();
	std::string str_index = std::to_string(index);
	int num_leading_zeroes = num_ply_chars - str_index.length();
	for (int i = 0; i < num_leading_zeroes; i++) {ply_location += '0';}
	ply_location += str_index;
	ply_location += ".pcd";

	return ply_location;
}

// int main() {
// 	VolumePkg vpkg = VolumePkg("/Users/david/Desktop//volumepkg/");
// 	std::cout << "Number of Slices: " << vpkg.getNumberOfSlices() << std::endl;
// 	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//     cv::imshow( "Display window", vpkg.getSliceAtIndex(1) );
//     cv::waitKey(0); 
// }
