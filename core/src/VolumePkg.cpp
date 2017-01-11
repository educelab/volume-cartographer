/** @file volumepkg.cpp */

#include <iostream>

#include <boost/range/iterator_range.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "core/io/OBJWriter.h"
#include "core/io/PLYWriter.h"
#include "core/io/PointSetIO.h"
#include "core/types/OrderedPointSet.h"
#include "core/types/VolumePkg.h"

namespace fs = boost::filesystem;

// CONSTRUCTORS //
// Make a volpkg of a particular version number
VolumePkg::VolumePkg(fs::path fileLocation, int version)
    : rootDir_{fileLocation}
    , segsDir_{fileLocation / "paths"}
    , sliceDir_{fileLocation / "slices"}
{
    // Lookup the metadata template from our library of versions
    auto findDict = volcart::VERSION_LIBRARY.find(version);
    if (findDict == std::end(volcart::VERSION_LIBRARY)) {
        throw std::runtime_error("No dictionary found for volpkg");
    }

    // Create the directories with the default values
    // TODO(cparker): We need a better way of handling default values
    config_ = VolumePkg::InitConfig(findDict->second, version);
    config_.set("slice location", "/slices/");

    // Make directories
    for (const auto& d : {rootDir_, segsDir_, sliceDir_}) {
        if (!fs::exists(d)) {
            fs::create_directory(d);
        }
    }

    // Initialize volume object
    vol_ = volcart::Volume(
        sliceDir_, config_.get<int>("number of slices"),
        config_.get<int>("width"), config_.get<int>("height"));
}

// Use this when reading a volpkg from a file
VolumePkg::VolumePkg(fs::path fileLocation)
    : rootDir_{fileLocation}
    , segsDir_{fileLocation / "paths"}
    , sliceDir_{fileLocation / "slices"}
{
    if (!(fs::exists(rootDir_) && fs::exists(segsDir_) &&
          fs::exists(sliceDir_))) {
        throw std::runtime_error("invalid volumepkg structure");
    }

    // Loads the metadata
    config_ = volcart::Metadata(fileLocation / "config.json");

    // Copy segmentation paths to segmentations_ vector
    auto range =
        boost::make_iterator_range(fs::directory_iterator(segsDir_), {});
    for (const auto& entry : range) {
        if (entry == "") {
            continue;
        }
        segmentations_.push_back(fs::basename(entry));
    }

    // Initialize volume object
    vol_ = volcart::Volume(
        sliceDir_, config_.get<int>("number of slices"),
        config_.get<int>("width"), config_.get<int>("height"));
}

// METADATA RETRIEVAL //
// Returns Volume Name from JSON config
std::string VolumePkg::getPkgName() const
{
    // Gets the Volume name from the configuration file
    auto name = config_.get<std::string>("volumepkg name");
    if (name != "NULL") {
        return name;
    } else {
        return "UnnamedVolume";
    }
}

int VolumePkg::getVersion() const { return config_.get<int>("version"); }

// Returns no. of slices from JSON config
int VolumePkg::getNumberOfSlices() const
{
    return config_.get<int>("number of slices");
}

int VolumePkg::getSliceWidth() const { return config_.get<int>("width"); }

int VolumePkg::getSliceHeight() const { return config_.get<int>("height"); }

double VolumePkg::getVoxelSize() const
{
    return config_.get<double>("voxelsize");
}

double VolumePkg::getMaterialThickness() const
{
    return config_.get<double>("materialthickness");
}

// Slice manipulation functions
bool VolumePkg::setSliceData(size_t index, const cv::Mat& slice)
{
    /**< Performs a read only check and then sets the data*/
    if (readOnly_) {
        volcart::ErrReadonly();
        return false;
    } else {
        return vol_.setSliceData(index, slice);
    }
}

// SEGMENTATION FUNCTIONS //
// Make a new folder inside the volume package to house everything for this
// segmentation
// and push back the new segmentation into our vector of segmentations_
std::string VolumePkg::newSegmentation()
{
    // make a new dir based off the current date and time
    auto newSegName = volcart::DateTime();
    auto newPath = segsDir_ / newSegName;

    // If the directory is successfully created, adds the name of the
    // segementation to the list
    if (fs::create_directory(newPath)) {
        segmentations_.push_back(newSegName);
    }

    return newSegName;
}

// Return a vector of strings representing the names of segmentations_ in the
// volpkg
std::vector<std::string> VolumePkg::getSegmentations() const
{
    return segmentations_;
}

// Set the private variable activeSeg_ to the seg we want to work with
void VolumePkg::setActiveSegmentation(const std::string& id)
{
    // TODO(cparker): Check that this seg actually exists in the volume
    activeSeg_ = id;
}

// Return the id of the active segmentation
std::string VolumePkg::getActiveSegmentation() { return activeSeg_; };

fs::path VolumePkg::getActiveSegPath() { return segsDir_ / activeSeg_; }

// Return the point cloud currently on disk for the activeSegmentation
volcart::OrderedPointSet<cv::Vec3d> VolumePkg::openCloud() const
{
    // TODO(cparker): Error if activeSeg_ not set
    auto outputName = segsDir_ / activeSeg_ / "pointset.vcps";
    return volcart::PointSetIO<cv::Vec3d>::ReadOrderedPointSet(
        outputName.string());
}

// Return the path to the active segmentation's mesh
fs::path VolumePkg::getMeshPath() const
{
    return segsDir_ / activeSeg_ / "cloud.ply";
}

// Return the texture image as a CV mat
cv::Mat VolumePkg::getTextureData() const
{
    auto texturePath = segsDir_ / activeSeg_ / "textured.png";
    return cv::imread(texturePath.string(), -1);
}

// Save a point cloud back to the volumepkg
int VolumePkg::saveCloud(const volcart::OrderedPointSet<cv::Vec3d>& ps) const
{
    auto outputName = segsDir_ / activeSeg_ / "pointset.vcps";
    std::cerr << "volcart::volpkg::Writing point cloud to file..." << std::endl;
    volcart::PointSetIO<cv::Vec3d>::WriteOrderedPointSet(
        outputName.string(), ps);
    std::cerr << "volcart::volpkg::Point cloud saved." << std::endl;
    return EXIT_SUCCESS;
}

int VolumePkg::saveMesh(const volcart::ITKMesh::Pointer& mesh) const
{
    fs::path outputName = segsDir_ / activeSeg_ / "cloud.ply";
    // Creates a PLY writer type and then writes the mesh out to the file
    volcart::io::PLYWriter writer(outputName, mesh);
    writer.write();
    return EXIT_SUCCESS;
}

void VolumePkg::saveMesh(
    const volcart::ITKMesh::Pointer& mesh,
    const volcart::Texture& texture) const
{
    // Creates an OBJ writer type and then writes the mesh and the texture out
    // to the file
    volcart::io::OBJWriter writer;
    auto meshPath = segsDir_ / activeSeg_ / "textured.obj";
    writer.setPath(meshPath);
    writer.setMesh(mesh);
    writer.setTexture(texture.image(0));
    writer.setUVMap(texture.uvMap());
    writer.write();
}

void VolumePkg::saveTextureData(const cv::Mat& texture, const std::string& name)
{
    auto texturePath = segsDir_ / activeSeg_ / (name + ".png");
    cv::imwrite(texturePath.string(), texture);
    std::cout << "Texture image saved" << std::endl;
}

volcart::Metadata VolumePkg::InitConfig(
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
        switch (entry.second) {
            case volcart::Type::INT:
                config.set(entry.first, int{});
                break;
            case volcart::Type::DOUBLE:
                config.set(entry.first, double{});
                break;
            case volcart::Type::STRING:
                config.set(entry.first, std::string{});
                break;
        }
    }

    return config;
}
