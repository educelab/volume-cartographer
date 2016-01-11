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
    segs_dir = file_location + "/paths/";
    slice_dir = file_location + "/slices/";
    config.setValue("slice location", "/slices/"); // To-Do: We need a better way of handling default values
    norm_dir = file_location + "/surface_normals/";

    // Initialize volume object
    vol_ = volcart::Volume(
            file_location,
            config.getInt("number of slices"),
            config.getInt("width"),
            config.getInt("height"));
};

// Use this when reading a volpkg from a file
VolumePkg::VolumePkg(std::string file_location) : config(file_location + "/config.json") {
    root_dir = file_location;
    segs_dir = file_location + "/paths/";
    slice_dir = file_location + "/slices/";
    norm_dir = file_location + "/surface_normals/";

    //iterate over paths in segs_dir, push_back to segmentations
    for(boost::filesystem::directory_iterator iter(segs_dir), end; iter != end; ++iter)
    {
      std::string path = boost::filesystem::basename(iter->path());
      if (path != "" ) segmentations.push_back(path);
    }

    // Initialize volume object
    vol_ = volcart::Volume(
            file_location,
            config.getInt("number of slices"),
            config.getInt("width"),
            config.getInt("height"));
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
    std::string name = config.getString("volumepkg name");
    if ( name != "NULL" )
        return name;
    else
        return "UnnamedVolume";
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
    config.save(filePath);
}

// Alias for saving to the default config.json
void VolumePkg::saveMetadata() {
    saveMetadata(root_dir.string() + "/config.json");
}

// Slice manipulation functions
bool VolumePkg::setSliceData(const size_t index, const cv::Mat& slice)
{
    if (_readOnly) {
        VC_ERR_READONLY();
        return false;
    } else {
        return vol_.setSliceData(index, slice);
    }
}

// SEGMENTATION FUNCTIONS //
// Return a vector of strings representing the names of segmentations in the volpkg
std::vector<std::string> VolumePkg::getSegmentations() {
    return segmentations;
}

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
    std::string segName = VC_DATE_TIME();
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

// Return the path to the active segmentation's mesh
std::string VolumePkg::getMeshPath(){
    std::string meshName = segs_dir.string() + "/" + activeSeg + "/cloud.ply";
    return meshName;
}

// Return the texture image as a CV mat
cv::Mat VolumePkg::getTextureData() {
    std::string texturePath = segs_dir.string() + "/" + activeSeg + "/textured.png";
    return cv::imread( texturePath, -1 );
}

// Save a point cloud back to the volumepkg
int VolumePkg::saveCloud(pcl::PointCloud<pcl::PointXYZRGB> segmentedCloud){
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/cloud.pcd";
    std::cerr << "volcart::volpkg::Writing point cloud to file..." << std::endl;
    try {
        pcl::io::savePCDFileBinaryCompressed(outputName, segmentedCloud);
    } catch(pcl::IOException) {
        std::cerr << "volcart::volpkg::error: Problem writing point cloud to file." << std::endl;
        return EXIT_FAILURE;
    }
    std::cerr << "volcart::volpkg::Point cloud saved." << std::endl;
    return EXIT_SUCCESS;
}

int VolumePkg::saveMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud) {
    std::string outputName = segs_dir.string() + "/" + activeSeg + "/cloud.ply";
    if ( volcart::meshing::orderedPCDMesher(segmentedCloud, outputName) == EXIT_SUCCESS ) {
        std::cerr << "volcart::volpkg::Mesh file saved." << std::endl;
        return EXIT_SUCCESS;
    } else {
        std::cerr << "volcart::volpkg::error: Problem writing mesh to file." << std::endl;
        return EXIT_FAILURE;
    }
}

void VolumePkg::saveMesh(VC_MeshType::Pointer mesh, volcart::Texture texture) {
    volcart::io::objWriter writer;
    writer.setPath(segs_dir.string() + "/" + activeSeg + "/textured.obj");
    writer.setMesh(mesh);
    writer.setTexture(texture.getImage(0));
    writer.setUVMap(texture.uvMap());
    writer.write();
};

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
