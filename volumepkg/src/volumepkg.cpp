/** @file volumepkg.cpp */

#include "volumepkg/volumepkg.h"
#include "common/io/PointSetIO.h"
#include "common/io/objWriter.h"
#include "common/io/plyWriter.h"
#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"

namespace fs = boost::filesystem;

// CONSTRUCTORS //
// Make a volpkg of a particular version number
VolumePkg::VolumePkg(
    const fs::path& file_location,
    int version)  // Changed type from double to int
{
    // Lookup the metadata template from our library of versions
    auto findDict = volcart::VersionLibrary.find(version);
    if (findDict == std::end(volcart::VersionLibrary)) {
        throw std::runtime_error("No dictionary found for volpkg");
    } else {
        config = VolumePkg::_initConfig(findDict->second, version);
    }

    // Create the directories with the default values
    // TODO: We need a better way of handling default values
    root_dir = file_location;
    segs_dir = file_location / "paths";
    slice_dir = file_location / "slices";
    config.set("slice location", "/slices/");

    // Initialize volume object
    vol_ = volcart::Volume(
        slice_dir, config.get<int>("number of slices"),
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
    // Loads the metadata
    config = volcart::Metadata(file_location / "config.json");

    segs_dir = file_location / "paths";
    slice_dir = file_location / "slices";
    if (!(fs::exists(segs_dir) || fs::exists(slice_dir))) {
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
    vol_ = volcart::Volume(
        slice_dir, config.get<int>("number of slices"),
        config.get<int>("width"), config.get<int>("height"));
};

// WRITE TO DISK //
int VolumePkg::initialize()
{
    // A check to see if the file can be written to disk
    if (_readOnly) {
        volcart::ERR_READONLY();
    }

    // Build the directory tree
    _makeDirTree();

    // Save the JSON to disk
    saveMetadata();

    return EXIT_SUCCESS;
};

int VolumePkg::_makeDirTree()
{
    // Directories we need to make
    std::vector<boost::filesystem::path> dirs;
    dirs.push_back(root_dir);
    dirs.push_back(segs_dir);
    dirs.push_back(slice_dir);

    // Make directories that don't exist
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
    // Gets the Volume name from the configuration file
    std::string name = config.get<std::string>("volumepkg name");
    if (name != "NULL")
        return name;
    else
        return "UnnamedVolume";
};

int VolumePkg::getVersion() const
{
    return config.get<int>("version");
};  // Changed type from double to int

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
void VolumePkg::saveMetadata(const fs::path& filePath)
{
    config.save(filePath);
}

// Alias for saving to the default config.json
void VolumePkg::saveMetadata() { saveMetadata(root_dir / "config.json"); }

// Slice manipulation functions
bool VolumePkg::setSliceData(size_t index, const cv::Mat& slice)
{
    /**< Performs a read only check and then sets the data*/
    if (_readOnly) {
        volcart::ERR_READONLY();
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
    auto newSegName = volcart::DATE_TIME();
    auto newPath = segs_dir / newSegName;

    // If the directory is successfully created, adds the name of the
    // segementation to the list
    if (fs::create_directory(newPath)) {
        segmentations.push_back(newSegName);
    }

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
    // TODO: Check that this seg actually exists in the volume
    activeSeg = name;
}

// Return the id of the active segmentation
std::string VolumePkg::getActiveSegmentation() { return activeSeg; };

boost::filesystem::path VolumePkg::getActiveSegPath()
{
    return segs_dir / activeSeg;
};

// Return the point cloud currently on disk for the activeSegmentation
volcart::OrderedPointSet<volcart::Point3d> VolumePkg::openCloud() const
{
    // TODO: Error if activeSeg not set
    auto outputName = segs_dir / activeSeg / "pointset.vcps";
    return volcart::PointSetIO<volcart::Point3d>::ReadOrderedPointSet(
        outputName.string());
}

// Return the path to the active segmentation's mesh
fs::path VolumePkg::getMeshPath() const
{
    return segs_dir / activeSeg / "cloud.ply";
}

// Return the texture image as a CV mat
cv::Mat VolumePkg::getTextureData() const
{
    auto texturePath = segs_dir / activeSeg / "textured.png";
    return cv::imread(texturePath.string(), -1);
}

// Save a point cloud back to the volumepkg
int VolumePkg::saveCloud(
    const volcart::OrderedPointSet<volcart::Point3d>& segmentedCloud) const
{
    auto outputName = segs_dir / activeSeg / "pointset.vcps";
    std::cerr << "volcart::volpkg::Writing point cloud to file..." << std::endl;
    volcart::PointSetIO<volcart::Point3d>::WriteOrderedPointSet(
        outputName.string(), segmentedCloud);
    std::cerr << "volcart::volpkg::Point cloud saved." << std::endl;
    return EXIT_SUCCESS;
}

int VolumePkg::saveMesh(const volcart::ITKMesh::Pointer mesh) const
{
    fs::path outputName = segs_dir / activeSeg / "cloud.ply";
    // Creates a PLY writer type and then writes the mesh out to the file
    volcart::io::plyWriter writer(outputName, mesh);
    writer.write();
    return EXIT_SUCCESS;
}

void VolumePkg::saveMesh(
    const volcart::ITKMesh::Pointer mesh, const volcart::Texture& texture) const
{
    // Creates an OBJ writer type and then writes the mesh and the texture out
    // to the file
    volcart::io::objWriter writer;
    auto meshPath = segs_dir / activeSeg / "textured.obj";
    writer.setPath(meshPath);
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
    const volcart::Dictionary& dict, int version)
{
    volcart::Metadata config;

    // Populate the config file with keys from the dictionary
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
