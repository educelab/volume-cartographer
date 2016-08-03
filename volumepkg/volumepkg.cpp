#include "volumepkg.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace fs = boost::filesystem;

// CONSTRUCTORS //
// Make a volpkg of a particular version number
VolumePkg::VolumePkg(const fs::path& file_location, double version)
{
    // Lookup the metadata template from our library of versions
    auto findDict = volcart::VersionLibrary.find(version);
    if (findDict == std::end(volcart::VersionLibrary)) {
        throw std::runtime_error("No dictionary found for volpkg");
    } else {
        config = VolumePkg::_initConfig(findDict->second, version);
    }

    root_dir = file_location;
    segs_dir = file_location / "paths";
    slice_dir = file_location / "slices";
    config.set(
        "slice location",
        "/slices/");  // To-Do: We need a better way of handling default values
    norm_dir = file_location / "surface_normals";

    // Initialize volume object
    vol_ = volcart::Volume(slice_dir, norm_dir,
                           config.get<int>("number of slices"),
                           config.get<int>("width"), config.get<int>("height"));
};

// Use this when reading a volpkg from a file
VolumePkg::VolumePkg(const fs::path& file_location)
{
    root_dir = file_location;
    if (!fs::exists(root_dir)) {
        auto errmsg = "location " + file_location.string() + " does not exist";
        throw std::runtime_error(errmsg);
    }

    config = volcart::Metadata(file_location / "config.json");

    segs_dir = file_location / "paths";
    slice_dir = file_location / "slices";
    norm_dir = file_location / "surface_normals";
    if (!(fs::exists(segs_dir) || fs::exists(slice_dir) ||
          fs::exists(norm_dir))) {
        auto errmsg = "invalid volumepkg structure";
        throw std::runtime_error(errmsg);
    }

    // Copy segmentation paths to segmentations vector
    auto range =
        boost::make_iterator_range(fs::directory_iterator(segs_dir), {});
    for (const auto& entry : range) {
        if (entry == "") {
            continue;
        }
        segmentations.push_back(fs::basename(entry));
    }

    // Initialize volume object
    vol_ = volcart::Volume(slice_dir, norm_dir,
                           config.get<int>("number of slices"),
                           config.get<int>("width"), config.get<int>("height"));
};

// WRITE TO DISK //
int VolumePkg::initialize()
{
    if (_readOnly) {
        VC_ERR_READONLY();
    }

    // Build the directory tree
    _makeDirTree();

    // Save the JSON to disk
    saveMetadata();

    return EXIT_SUCCESS;
};

int VolumePkg::_makeDirTree()
{
    // Dirs we need to make
    std::vector<boost::filesystem::path> dirs;
    dirs.push_back(root_dir);
    dirs.push_back(segs_dir);
    dirs.push_back(slice_dir);
    dirs.push_back(norm_dir);

    // Make dirs that don't exist
    for (auto dir = dirs.begin(); dir != dirs.end(); ++dir) {
        if (boost::filesystem::exists(*dir))
            std::cerr << "WARNING: " << *dir
                      << " already exists. Skipping folder creation."
                      << std::endl;
        else
            boost::filesystem::create_directory(*dir);
    }

    return EXIT_SUCCESS;
};

// METADATA RETRIEVAL //
// Returns Volume Name from JSON config
std::string VolumePkg::getPkgName() const
{
    std::string name = config.get<std::string>("volumepkg name");
    if (name != "NULL")
        return name;
    else
        return "UnnamedVolume";
};

double VolumePkg::getVersion() const { return config.get<double>("version"); };

// Returns no. of slices from JSON config
int VolumePkg::getNumberOfSlices() const
{
    return config.get<int>("number of slices");
}

int VolumePkg::getSliceWidth() const { return config.get<int>("width"); }

int VolumePkg::getSliceHeight() const { return config.get<int>("height"); }

double VolumePkg::getVoxelSize() const
{
    return config.get<double>("voxelsize");
}

double VolumePkg::getMaterialThickness() const
{
    return config.get<double>("materialthickness");
}

// METADATA EXPORT //
// Save metadata to any file
void VolumePkg::saveMetadata(const std::string& filePath)
{
    config.save(filePath);
}

// Alias for saving to the default config.json
void VolumePkg::saveMetadata()
{
    saveMetadata(root_dir.string() + "/config.json");
}

// Slice manipulation functions
bool VolumePkg::setSliceData(size_t index, const cv::Mat& slice)
{
    if (_readOnly) {
        VC_ERR_READONLY();
        return false;
    } else {
        return vol_.setSliceData(index, slice);
    }
}

// SEGMENTATION FUNCTIONS //
// Make a new folder inside the volume package to house everything for this
// segmentation
// and push back the new segmentation into our vector of segmentations
std::string VolumePkg::newSegmentation()
{
    // make a new dir based off the current date and time
    auto newSegName = VC_DATE_TIME();
    auto newPath = segs_dir / newSegName;

    if (fs::create_directory(newPath)) {
        segmentations.push_back(newSegName);
    };

    return newSegName;
}

// Return a vector of strings representing the names of segmentations in the
// volpkg
std::vector<std::string> VolumePkg::getSegmentations() const
{
    return segmentations;
}

// Set the private variable activeSeg to the seg we want to work with
void VolumePkg::setActiveSegmentation(const std::string& name)
{
    // To-Do: Check that this seg actually exists in the volume
    activeSeg = name;
}

// Return the id of the active segmentation
std::string VolumePkg::getActiveSegmentation() {
    return activeSeg;
};

boost::filesystem::path VolumePkg::getActiveSegPath() {
    return segs_dir / activeSeg;
};

// Return the point cloud currently on disk for the activeSegmentation
pcl::PointCloud<pcl::PointXYZRGB>::Ptr VolumePkg::openCloud() const
{
    // To-Do: Error if activeSeg not set
    auto outputName = segs_dir / activeSeg / "cloud.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(outputName.string(), *cloud);
    return cloud;
}

// Return the path to the active segmentation's mesh
std::string VolumePkg::getMeshPath() const
{
    return (segs_dir / activeSeg / "cloud.ply").string();
}

// Return the texture image as a CV mat
cv::Mat VolumePkg::getTextureData() const
{
    auto texturePath = segs_dir / activeSeg / "textured.png";
    return cv::imread(texturePath.string(), -1);
}

// Save a point cloud back to the volumepkg
int VolumePkg::saveCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>& segmentedCloud) const
{
    auto outputName = segs_dir / activeSeg / "cloud.pcd";
    std::cerr << "volcart::volpkg::Writing point cloud to file..." << std::endl;
    try {
        pcl::io::savePCDFileBinaryCompressed(outputName.string(),
                                             segmentedCloud);
    } catch (pcl::IOException) {
        std::cerr
            << "volcart::volpkg::error: Problem writing point cloud to file."
            << std::endl;
        return EXIT_FAILURE;
    }
    std::cerr << "volcart::volpkg::Point cloud saved." << std::endl;
    return EXIT_SUCCESS;
}

int VolumePkg::saveMesh(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segmentedCloud) const
{
    auto outputName = segs_dir / activeSeg / "cloud.ply";
    if (volcart::meshing::orderedPCDMesher(
            segmentedCloud, outputName.string()) == EXIT_SUCCESS) {
        std::cerr << "volcart::volpkg::Mesh file saved." << std::endl;
        return EXIT_SUCCESS;
    } else {
        std::cerr << "volcart::volpkg::error: Problem writing mesh to file."
                  << std::endl;
        return EXIT_FAILURE;
    }
}

void VolumePkg::saveMesh(const VC_MeshType::Pointer& mesh,
                         volcart::Texture& texture) const
{
    volcart::io::objWriter writer;
    auto meshPath = segs_dir / activeSeg / "textured.obj";
    writer.setPath(meshPath.string());
    writer.setMesh(mesh);
    writer.setTexture(texture.getImage(0));
    writer.setUVMap(texture.uvMap());
    writer.write();
};

void VolumePkg::saveTextureData(const cv::Mat& texture, const std::string& name)
{
    auto texturePath = segs_dir / activeSeg / (name + ".png");
    cv::imwrite(texturePath.string(), texture);
    printf("Texture image saved.\n");
}

volcart::Metadata VolumePkg::_initConfig(
    const volcart::Dictionary& dict,
    double version)
{
    volcart::Metadata config;

    // Populate the cfg with keys from the dict
    for (const auto& entry : dict) {
        if (entry.first == "version") {
            config.set("version", version);
            continue;
        }

        // Default values
        if (entry.second == "int") {
            config.set(entry.first, int{});
        } else if (entry.second == "double") {
            config.set(entry.first, double{});
        } else {
            config.set(entry.first, std::string{});
        }
    }

    return config;
}
