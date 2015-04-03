#include "volumepkg.h"

VolumePkg::VolumePkg(std::string file_location) : config(file_location + "/config.json"){
    location = file_location;
}

// Returns # of slices from JSON config
int VolumePkg::getNumberOfSlices() {
    return config.getInt("number of slices");
}

// Returns Volume Name from JSON config
std::string VolumePkg::getPkgName() {
    return config.getString("volumepkg name", "UnnamedVolume");
}

// Returns # of significant digits for # of slices
int VolumePkg::getNumberOfSliceCharacters() {
    int num_slices = getNumberOfSlices();
    int num_characters = 0;
    while (num_slices > 0) {
        num_characters += 1;
        num_slices /= 10;
    }
    return num_characters;
}

// Returns slice at specific slice index
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

    cv::Mat sliceImg = cv::imread( slice_location, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
    
    return sliceImg;
}

// Returns surface normal PCD file path for slice at index
std::string VolumePkg::getNormalAtIndex(int index) {

    std::string pcd_location(location);
    pcd_location += config.getString("pcd location", "/surface_normals/");

    int num_pcd_chars = getNumberOfSliceCharacters();
    std::string str_index = std::to_string(index);
    int num_leading_zeroes = num_pcd_chars - str_index.length();
    for (int i = 0; i < num_leading_zeroes; i++) {pcd_location += '0';}
    pcd_location += str_index;
    pcd_location += ".pcd";

    return pcd_location;
}

// To-Do: Return a vector of ints representing the number of segmentations in the volpkg
std::vector<int> getSegmentations(){};

// To-Do: Set the private variable activeSeg to the seg we want to work with
int setActiveSegmentation(int){};

// To-Do - Needs to make a new folder inside the volume package to house everything for this segmentation
int VolumePkg::newSegmentation() {
    //get the file name
    std::string slice_location(location);
    slice_location += config.getString("segpath", "/paths/");
}

// To-Do: Return the point cloud currently on disk for the activeSegmentation
pcl::PointCloud<pcl::PointXYZRGB> openCloud(){};

// To-Do: Update the point cloud on the disk
int saveCloud(pcl::PointCloud<pcl::PointXYZRGB>){};