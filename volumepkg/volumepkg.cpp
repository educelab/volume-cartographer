#include "volumepkg.h"

VolumePkg::VolumePkg(std::string file_location) : config(file_location + "/config.json") {
    location = file_location;
    segdir = file_location + config.getString("segpath", "/paths/");
    //iterate over paths in segdir, push_back to segmentations
    for(boost::filesystem::directory_iterator iter(segdir), end; iter != end; ++iter)
    {
      std::string path = boost::filesystem::basename(iter->path());
      if (path != "" ) segmentations.push_back(path);
    }
}

// DEBUG FUNCTIONS //
// Print the currently stored PicoJson Object
void VolumePkg::printObject() {
    config.printObject();
}

// METADATA RETRIEVAL //
// Returns Volume Name from JSON config
std::string VolumePkg::getPkgName() {
    return config.getString("volumepkg name", "UnnamedVolume");
}

double VolumePkg::getVersion() {
    return config.getDouble("version");
};

// Returns no. of slices from JSON config
int VolumePkg::getNumberOfSlices() {
    return config.getInt("number of slices");
}

int VolumePkg::getSliceWidth() {
    return config.getInt("width");
}

int VolumePkg::getSliceHeight() {
    return config.getInt("height");
}

double VolumePkg::getVoxelSize() {
    return config.getDouble("voxelsize");
};

double VolumePkg::getMaterialThickness() {
    return config.getDouble("materialthickness");
};

// METADATA ASSIGNMENT //
int VolumePkg::setMetadata(std::string key, int value) {
    std::string keyType = findKeyType(key);
    if (keyType == "int") {
        config.setValue(key, value);
        return EXIT_SUCCESS;
    }
    else if (keyType == "") {
        return EXIT_FAILURE;
    }
    else {
        std::cerr << "ERROR: Value \"" << value << "\" not of type specified by dictionary (" << keyType << ")" << std::endl;
        return EXIT_FAILURE;
    }
}

int VolumePkg::setMetadata(std::string key, double value) {
    std::string keyType = findKeyType(key);
    if (keyType == "double") {
        config.setValue(key, value);
        return EXIT_SUCCESS;
    }
    else if (keyType == "") {
        return EXIT_FAILURE;
    }
    else {
        std::cerr << "ERROR: Value \"" << value << "\" not of type specified by dictionary (" << keyType << ")" << std::endl;
        return EXIT_FAILURE;
    }
}

int VolumePkg::setMetadata(std::string key, std::string value) {
    std::string keyType = findKeyType(key);
    if (keyType == "string") {
        config.setValue(key, value);
        return EXIT_SUCCESS;
    }
    else if (keyType == "int") {
        try {
            int castValue = boost::lexical_cast<int>(value);
            config.setValue(key, castValue);
            return EXIT_SUCCESS;
        }
        catch(const boost::bad_lexical_cast &) {
            std::cerr << "ERROR: Given value \"" << value << "\" cannot be cast to type specified by dictionary (" << keyType << ")" << std::endl;
            return EXIT_FAILURE;
        }
    }
    else if (keyType == "double") {
        try {
            double castValue = boost::lexical_cast<double>(value);
            config.setValue(key, castValue);
            return EXIT_SUCCESS;
        }
        catch(const boost::bad_lexical_cast &) {
            std::cerr << "ERROR: Given value \"" << value << "\" cannot be cast to type specified by dictionary (" << keyType << ")" << std::endl;
            return EXIT_FAILURE;
        }
    }
    else if (keyType == "") {
        return EXIT_FAILURE;
    }
    else {
        std::cerr << "ERROR: Value \"" << value << "\" not of type specified by dictionary (" << keyType << ")" << std::endl;
        return EXIT_FAILURE;
    }
}


// METADATA EXPORT //
// Save metadata to volpkg config.json
void VolumePkg::saveMetadata() {
    std::string filePath = location + "/config.json";
    config.saveCfg(filePath);
}

// Save metadata to any file
void VolumePkg::saveMetadata(std::string filePath) {
    config.saveCfg(filePath);
}


// DATA RETRIEVAL //
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

// Returns slice image at specific slice index
cv::Mat VolumePkg::getSliceData(int index) {
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

// Returns slice at specific slice index
std::string VolumePkg::getSlicePath(int index) {
    //get the file name
    std::string slice_location(location);
    slice_location += config.getString("slice location", "/slices/");
    int num_slice_characters = getNumberOfSliceCharacters();
    std::string str_index = std::to_string(index);
    int num_leading_zeroes = num_slice_characters - str_index.length();
    for (int i = 0; i < num_leading_zeroes; i++) {slice_location += '0';}
    slice_location += str_index;
    slice_location += ".tif";

    return slice_location;
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


// SEGMENTATION FUNCTIONS //
// Return a vector of strings representing the names of segmentations in the volpkg
std::vector<std::string> VolumePkg::getSegmentations() {
    return segmentations;
};

// Set the private variable activeSeg to the seg we want to work with
void VolumePkg::setActiveSegmentation(std::string name) {
    // To-Do: Check that this seg actually exists in the volume
    activeSeg = name;
};

// Make a new folder inside the volume package to house everything for this segmentation
// and push back the new segmentation into our vector of segmentations
std::string VolumePkg::newSegmentation() {
    //get the file name
    boost::filesystem::path newSeg(segdir);

    //make a new dir based off the current date and time
    time_t now = time( 0 );
    struct tm tstruct;
    char buf[ 80 ];
    tstruct = *localtime( &now );
    strftime( buf, sizeof( buf ), "%Y%m%d%H%M%S", &tstruct );
    std::string segName(buf);
    newSeg += segName;

    if (boost::filesystem::create_directory(newSeg)) {
        segmentations.push_back(segName);
    };
  
  return segName;
}

// Return the point cloud currently on disk for the activeSegmentation
pcl::PointCloud<pcl::PointXYZRGB>::Ptr VolumePkg::openCloud() {
    // To-Do: Error if activeSeg not set
    std::string outputName = segdir.string() + "/" + activeSeg + "/cloud.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (outputName, *cloud);
    return cloud;
}

// Return the untextured mesh from the volpkg
ChaoVis::CMesh VolumePkg::openMesh() {
    ChaoVis::CMesh mesh;
    std::string outputName = segdir.string() + "/" + activeSeg + "/cloud.ply";
    ChaoVis::CPlyHelper::ReadPlyFile( outputName, mesh );
    std::cout << "Mesh file loaded." << std::endl;
    return mesh;
}

// Return the textured mesh from the volpkg
ChaoVis::CMesh VolumePkg::openTexturedMesh() {
    ChaoVis::CMesh mesh;
    std::string outputName = segdir.string() + "/" + activeSeg + "/textured.ply";
    ChaoVis::CPlyHelper::ReadPlyFile( outputName, mesh );
    std::cout << "Mesh file loaded." << std::endl;
    return mesh;
}

// Return the path to the active segmentation's mesh
std::string VolumePkg::getMeshPath(){
    std::string meshName = segdir.string() + "/" + activeSeg + "/cloud.ply";
    return meshName;
}

// Return the texture image as a CV mat
cv::Mat VolumePkg::getTextureData() {
    std::string texturePath = segdir.string() + "/" + activeSeg + "/texture.tif";
    cv::Mat texture = cv::imread( texturePath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
    return cv::imread( texturePath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
}

// Save a point cloud back to the volumepkg
void VolumePkg::saveCloud(pcl::PointCloud<pcl::PointXYZRGB> segmentedCloud){
    std::string outputName = segdir.string() + "/" + activeSeg + "/cloud.pcd";
    printf("Writing point cloud to file...\n");
    pcl::io::savePCDFileBinaryCompressed(outputName, segmentedCloud);
    printf("Point cloud saved.\n");
}

void VolumePkg::saveMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud) {
    std::string outputName = segdir.string() + "/" + activeSeg + "/cloud.ply";
    orderedPCDMesher(segmentedCloud, outputName);
    printf("Mesh file saved.\n");
}

void VolumePkg::saveTexturedMesh(ChaoVis::CMesh mesh) {
    std::string outputName = segdir.string() + "/" + activeSeg + "/textured.ply";
    ChaoVis::CPlyHelper::WritePlyFile( outputName, mesh );
    printf("Mesh file saved.\n");
}

void VolumePkg::saveTextureData(cv::Mat texture, std::string name){
    std::string texturePath = segdir.string() + "/" + activeSeg + "/" + name + ".png";
    cv::imwrite(texturePath, texture);
    printf("Texture image saved.\n");
}

// See if the given key exists in the volumepkg dictionary and return its type
std::string VolumePkg::findKeyType(std::string key) {
    std::unordered_map <double, VolCart::VersionDict>::const_iterator vFind = VolCart::versionsList.find(this->getVersion());
    if ( vFind == VolCart::versionsList.end() ) {
        std::cerr << "ERROR: No dictionary found for volpkg v." << this->getVersion() << std::endl;
        return "";
    }
    else {
        VolCart::VersionDict dict = vFind->second;
        std::unordered_map <std::string, std::string>::const_iterator kFind = dict.find(key);
        if ( kFind == dict.end() ) {
            std::cerr << "ERROR: Key \"" << key << "\" not found in dictionary for volpkg v." << this->getVersion() << std::endl;
            return "";
        }
        else {
            return kFind->second;
        }
    }
}
