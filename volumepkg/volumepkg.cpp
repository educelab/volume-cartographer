#include "volumepkg.h"

// CONSTRUCTORS //
// Make a volpkg of a particular version number
VolumePkg::VolumePkg(std::string file_location, double version ) {

    // Lookup the metadata template from our library of versions
    volcart::Library::const_iterator findDict = volcart::VersionLibrary.find(version);
    if ( findDict == volcart::VersionLibrary.end() ) {
        std::cerr << "ERROR: No dictionary found for volpkg v." << version << std::endl;
        // To-Do: Throw an exception in the event we have no dictionary
    } else {
        config = VolumePkgCfg(findDict->second, version);
    }

    root_dir = file_location;
    segs_dir = file_location + config.getString("segpath", "/paths/");
    slice_dir = file_location + "/slices/";
    config.setValue("slice location", "/slices/"); // To-Do: We need a better way of handling default values
    norm_dir = file_location + "/surface_normals/";
};

// Use this when reading a volpkg from a file
VolumePkg::VolumePkg(std::string file_location) : config(file_location + "/config.json") {
    root_dir = file_location;
    segs_dir = file_location + config.getString("segpath", "/paths/");
    slice_dir = file_location + "/slices/";
    norm_dir = file_location + "/surface_normals/";

    //iterate over paths in segs_dir, push_back to segmentations
    for(boost::filesystem::directory_iterator iter(segs_dir), end; iter != end; ++iter)
    {
      std::string path = boost::filesystem::basename(iter->path());
      if (path != "" ) segmentations.push_back(path);
    }
};

// WRITE TO DISK //
int VolumePkg::initialize() {
    if (_readOnly) VC_ERR_READONLY();

    // Build the directory tree
    _makeDirTree();

    // Save the JSON to disk
    saveMetadata();

    return EXIT_SUCCESS;
};

int VolumePkg::_makeDirTree() {

    // Dirs we need to make
    std::vector<boost::filesystem::path> dirs;
    dirs.push_back(root_dir);
    dirs.push_back(segs_dir);
    dirs.push_back(slice_dir);
    dirs.push_back(norm_dir);

    // Make dirs that don't exist
    for ( auto dir = dirs.begin(); dir != dirs.end(); ++dir) {
        if (boost::filesystem::exists(*dir))
            std::cerr << "WARNING: " << *dir << " already exists. Skipping folder creation." << std::endl;
        else
            boost::filesystem::create_directory(*dir);
    }

    return EXIT_SUCCESS;
};

// METADATA RETRIEVAL //
// Returns Volume Name from JSON config
std::string VolumePkg::getPkgName() {
    return config.getString("volumepkg name", "UnnamedVolume");
};

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


// METADATA EXPORT //
// Save metadata to any file
void VolumePkg::saveMetadata(std::string filePath) {
    config.saveCfg(filePath);
}

// Alias for saving to the default config.json
void VolumePkg::saveMetadata() {
    saveMetadata(root_dir.string() + "/config.json");
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

    std::string filepath = getSlicePath(index);

    if ( boost::filesystem::exists(filepath) )
        return cv::imread( filepath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
//    else
//        // To-Do: Throw an exception/error
}

// Returns slice at specific slice index
std::string VolumePkg::getSlicePath(int index) {

    std::string filepath  = slice_dir.string();
    std::string index_str = std::to_string(index);

    int leading_zeroes = getNumberOfSliceCharacters() - index_str.length();
    for (int i = 0; i < leading_zeroes; ++i) filepath += '0';
    filepath += index_str + ".tif";

    return filepath;
}

// Returns surface normal PCD file path for slice at index
std::string VolumePkg::getNormalAtIndex(int index) {

    std::string pcd_location(root_dir.string());
    pcd_location += config.getString("pcd location", "/surface_normals/");

    int num_pcd_chars = getNumberOfSliceCharacters();
    std::string str_index = std::to_string(index);
    int num_leading_zeroes = num_pcd_chars - str_index.length();
    for (int i = 0; i < num_leading_zeroes; i++) {pcd_location += '0';}
    pcd_location += str_index;
    pcd_location += ".pcd";

    return pcd_location;
}


// DATA ASSIGNMENT //
int VolumePkg::setSliceData(unsigned long index, cv::Mat slice) {

    if (_readOnly) VC_ERR_READONLY();
    if ( index >= getNumberOfSlices() ) {
        std::cerr << "ERROR: Atttempted to save a slice image to an out of bounds index." << std::endl;
        return EXIT_FAILURE;
    }

    std::string filepath = getSlicePath(index);

    cv::imwrite(filepath, slice);

    return EXIT_SUCCESS;
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
    boost::filesystem::path newSeg(segs_dir);

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
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/cloud.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (outputName, *cloud);
    return cloud;
}

// Return the untextured mesh from the volpkg
ChaoVis::CMesh VolumePkg::openMesh() {
    ChaoVis::CMesh mesh;
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/cloud.ply";
    ChaoVis::CPlyHelper::ReadPlyFile( outputName, mesh );
    std::cout << "Mesh file loaded." << std::endl;
    return mesh;
}

// Return the textured mesh from the volpkg
ChaoVis::CMesh VolumePkg::openTexturedMesh() {
    ChaoVis::CMesh mesh;
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/textured.ply";
    ChaoVis::CPlyHelper::ReadPlyFile( outputName, mesh );
    std::cout << "Mesh file loaded." << std::endl;
    return mesh;
}

// Return the path to the active segmentation's mesh
std::string VolumePkg::getMeshPath(){
    std::string meshName = segs_dir.string() + "/" + activeSeg + "/cloud.ply";
    return meshName;
}

// Return the texture image as a CV mat
cv::Mat VolumePkg::getTextureData() {
    std::string texturePath = segs_dir.string() + "/" + activeSeg + "/texture.tif";
    cv::Mat texture = cv::imread( texturePath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
    return cv::imread( texturePath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );
}

// Save a point cloud back to the volumepkg
void VolumePkg::saveCloud(pcl::PointCloud<pcl::PointXYZRGB> segmentedCloud){
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/cloud.pcd";
    printf("Writing point cloud to file...\n");
    pcl::io::savePCDFileBinaryCompressed(outputName, segmentedCloud);
    printf("Point cloud saved.\n");
}

void VolumePkg::saveMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud) {
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/cloud.ply";
    volcart::meshing::orderedPCDMesher(segmentedCloud, outputName);
    printf("Mesh file saved.\n");
}

void VolumePkg::saveTexturedMesh(ChaoVis::CMesh mesh) {
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/textured.ply";
    ChaoVis::CPlyHelper::WritePlyFile( outputName, mesh );
    printf("Mesh file saved.\n");
}

void VolumePkg::saveTextureData(cv::Mat texture, std::string name){
    std::string texturePath = segs_dir.string() + "/" + activeSeg + "/" + name + ".png";
    cv::imwrite(texturePath, texture);
    printf("Texture image saved.\n");
}

// See if the given key exists in the volumepkg dictionary and return its type
std::string VolumePkg::findKeyType(std::string key) {
    volcart::Library::const_iterator vFind = volcart::VersionLibrary.find(this->getVersion());
    if ( vFind == volcart::VersionLibrary.end() ) {
        std::cerr << "ERROR: No dictionary found for volpkg v." << this->getVersion() << std::endl;
        return "";
    }
    else {
        volcart::Dictionary dict = vFind->second;
        volcart::Dictionary::const_iterator kFind = dict.find(key);
        if ( kFind == dict.end() ) {
            std::cerr << "ERROR: Key \"" << key << "\" not found in dictionary for volpkg v." << this->getVersion() << std::endl;
            return "";
        }
        else {
            return kFind->second;
        }
    }
}
