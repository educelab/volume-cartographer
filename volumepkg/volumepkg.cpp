#include <io/objWriter.h>
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

    // Take advantage of caching layer
    auto possibleSlice = cache.get(index);
    if (possibleSlice != nullptr) {
        return *possibleSlice;
    }

    cv::Mat sliceImg = cv::imread( getSlicePath(index), -1 );
    
    // Put into cache so we can use it later
    cache.put(index, sliceImg);
    
    return sliceImg;
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

    std::string pcd_location = norm_dir.string();

    int num_pcd_chars = getNumberOfSliceCharacters();
    std::string str_index = std::to_string(index);
    int num_leading_zeroes = num_pcd_chars - str_index.length();
    for (int i = 0; i < num_leading_zeroes; i++) {pcd_location += '0';}
    pcd_location += str_index;
    pcd_location += ".pcd";

    return pcd_location;
}

// Limit the number of elements in the cache
void VolumePkg::setCacheSize(size_t size) {
    cache.setSize(size);
}

// Limit the size of the cache in bytes
void VolumePkg::setCacheMemory(size_t size) {
    size_t slice_size = getSliceData(0).step[0] * getSliceData(0).rows;
    setCacheSize(size/slice_size);
};


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

//Reslice VolumePkg::reslice(const cv::Vec3f center, const cv::Vec3f xvec, const cv::Vec3f yvec,
//                           const int32_t width, const int32_t height) {
//    const auto xnorm = cv::normalize(xvec);
//    const auto ynorm = cv::normalize(yvec);
//    const auto origin = center - ((width / 2) * xnorm + (height / 2) * ynorm);
//
//    cv::Mat m(height, width, CV_16UC1);
//    for (int h = 0; h < height; ++h) {
//        for (int w = 0; w < width; ++w) {
//            cv::Vec3f v = origin + (h * ynorm) + (w * xnorm);
//            m.at<uint16_t>(h, w) = interpolateAt(v);
//        }
//    }
//
//    return Reslice(m, origin, xnorm, ynorm);
//}

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
uint16_t VolumePkg::interpolateAt(cv::Vec3f point) {
    double int_part;
    double dx = modf(point(VC_INDEX_X), &int_part);
    double dy = modf(point(VC_INDEX_Y), &int_part);
    double dz = modf(point(VC_INDEX_Z), &int_part);

    int x_min = (int) point(VC_INDEX_X);
    int x_max = x_min + 1;
    int y_min = (int) point(VC_INDEX_Y);
    int y_max = y_min + 1;
    int z_min = (int) point(VC_INDEX_Z);
    int z_max = z_min + 1;

    // insert safety net
    if (x_min < 0 || y_min < 0 || z_min < 0 ||
        x_max >= getSliceWidth() || y_max >= getSliceHeight() ||
        z_max >= getNumberOfSlices()) {
        return 0;
    }

    // Slice data for certain slices
    auto sliceZmin = getSliceData(z_min);
    auto sliceZmax = getSliceData(z_max);

    return uint16_t(
            sliceZmin.at<uint16_t>(y_min, x_min) * (1 - dx) * (1 - dy) * (1 - dz) +
            sliceZmin.at<uint16_t>(y_min, x_max) * dx       * (1 - dy) * (1 - dz) +
            sliceZmin.at<uint16_t>(y_max, x_min) * (1 - dx) * dy       * (1 - dz) +
            sliceZmax.at<uint16_t>(y_min, x_min) * (1 - dx) * (1 - dy) * dz +
            sliceZmax.at<uint16_t>(y_min, x_max) * dx       * (1 - dy) * dz +
            sliceZmax.at<uint16_t>(y_max, x_min) * (1 - dx) * dy       * dz +
            sliceZmin.at<uint16_t>(y_max, x_max) * dx       * dy       * (1 - dz) +
            sliceZmax.at<uint16_t>(y_max, x_max) * dx       * dy       * dz);
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
