#include "volumepkg.h"

VolumePkg::VolumePkg(std::string file_location) : config(file_location + "/config.json") {
    location = file_location;
    segdir = file_location + config.getString("segpath", "/paths/");
    boost::filesystem::directory_iterator it(segdir), eod;
    //iterate over paths in segdir, push_back to segmentations
    for(boost::filesystem::recursive_directory_iterator iter(segdir), end; iter != end; ++iter)
    {
      std::string path = iter->path().string();
      segmentations.push_back(path);
    }
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

// To-Do: Return a vector of strings representing the number of segmentations in the volpkg
std::vector<std::string> VolumePkg::getSegmentations() {
    return segmentations;
};

// To-Do: Set the private variable activeSeg to the seg we want to work with
void VolumePkg::setActiveSegmentation(std::string name) {
    activeSeg = name;
};

// To-Do - Needs to make a new folder inside the volume package to house everything for this segmentation
std::string VolumePkg::newSegmentation() {
    //get the file name
    boost::filesystem::path newSeg(segdir);

    //make a new dir based off the current date and time
    time_t now = time( 0 );
    struct tm tstruct;
    char buf[ 80 ];
    tstruct = *localtime( &now );
    int result = strftime( buf, sizeof( buf ), "%Y%m%d%H%M%S", &tstruct );
    std::string segName(buf);
    newSeg += segName;

    if (boost::filesystem::create_directory(newSeg)) {
        segmentations.push_back(segName);
    };
  
  return segName;
}

// To-Do: Return the point cloud currently on disk for the activeSegmentation
pcl::PointCloud<pcl::PointXYZRGB>::Ptr VolumePkg::openCloud() {
  std::string outputName = segdir.string() + "/" + activeSeg + "/" + getPkgName() + "-" + activeSeg + "_segmented.pcd";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB> (outputName, *cloud);
  return cloud;
};

// To-Do: Update the point cloud on the disk
void VolumePkg::saveCloud(pcl::PointCloud<pcl::PointXYZRGB> segmentedCloud){
  std::string outputName = segdir.string() + "/" + activeSeg + "/" + getPkgName() + "-" + activeSeg + "_segmented.pcd";
  std::cout << outputName << std::endl;
  printf("Writing point cloud to file...\n");
  pcl::io::savePCDFileASCII(outputName, segmentedCloud);
  printf("Segmentation complete!\n");
};