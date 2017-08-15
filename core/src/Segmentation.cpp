#include "vc/core/types/Segmentation.hpp"

#include "vc/core/io/PointSetIO.hpp"

using namespace volcart;

namespace fs = boost::filesystem;

// Load a Segmentation directory from disk
// Reads and verifies metadata
Segmentation::Segmentation(boost::filesystem::path path)
    : DiskBasedObjectBaseClass(path)
{
    if (metadata_.get<std::string>("type") != "seg") {
        throw std::runtime_error("File not of type: seg");
    }
}

// Make a new Segmentation file on disk
Segmentation::Segmentation(fs::path path, Identifier uuid, std::string name)
    : DiskBasedObjectBaseClass(path, uuid, name)
{
    metadata_.set("type", "seg");
    metadata_.set("vcps", std::string{});
    metadata_.save();
}

// Load a Segmentation from disk, return a pointer
Segmentation::Pointer Segmentation::New(fs::path path)
{
    return std::make_shared<Segmentation>(path);
}

// Make a new segmentation on disk, return a pointer
Segmentation::Pointer Segmentation::New(
    fs::path path, std::string uuid, std::string name)
{
    return std::make_shared<Segmentation>(path, uuid, name);
}

// Save the PointSet to disk
void Segmentation::setPointSet(const OrderedPointSet<cv::Vec3d>& ps)
{
    // Set a name into the metadata if we haven't set one already
    if (metadata_.get<std::string>("vcps").empty()) {
        metadata_.set("vcps", "pointset.vcps");
    }

    // Write the pointset to the segmentation file
    auto filepath = path_ / metadata_.get<std::string>("vcps");
    PointSetIO<cv::Vec3d>::WriteOrderedPointSet(filepath, ps);
}

// Load the PointSet from disk
OrderedPointSet<cv::Vec3d> Segmentation::getPointSet() const
{
    // Make sure there's an associated pointset file
    if (metadata_.get<std::string>("vcps").empty()) {
        throw std::runtime_error("segmentation has no pointset");
    }

    // Load the pointset
    auto filepath = path_ / metadata_.get<std::string>("vcps");
    return PointSetIO<cv::Vec3d>::ReadOrderedPointSet(filepath);
}