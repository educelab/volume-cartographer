#include "vc/core/types/Segmentation.hpp"

#include "vc/core/io/PointSetIO.hpp"

using namespace volcart;

namespace fs = volcart::filesystem;

// Load a Segmentation directory from disk
// Reads and verifies metadata
Segmentation::Segmentation(fs::path path)
    : DiskBasedObjectBaseClass(std::move(path))
{
    if (metadata_.get<std::string>("type") != "seg") {
        throw std::runtime_error("File not of type: seg");
    }
}

// Make a new Segmentation file on disk
Segmentation::Segmentation(fs::path path, Identifier uuid, std::string name)
    : DiskBasedObjectBaseClass(
          std::move(path), std::move(uuid), std::move(name))
{
    metadata_.set("type", "seg");
    metadata_.set("vcps", std::string{});
    metadata_.set("volume", Volume::Identifier{});
    metadata_.save();
}

// Load a Segmentation from disk, return a pointer
auto Segmentation::New(fs::path path) -> Segmentation::Pointer
{
    return std::make_shared<Segmentation>(path);
}

// Make a new segmentation on disk, return a pointer
auto Segmentation::New(fs::path path, std::string uuid, std::string name)
    -> Segmentation::Pointer
{
    return std::make_shared<Segmentation>(path, uuid, name);
}

// Save the PointSet to disk
void Segmentation::setPointSet(const PointSet& ps)
{
    // Set a name into the metadata if we haven't set one already
    if (metadata_.get<std::string>("vcps").empty()) {
        metadata_.set("vcps", "pointset.vcps");
        metadata_.save();
    }

    // Write the pointset to the segmentation file
    auto filepath = path_ / metadata_.get<std::string>("vcps");
    PointSetIO<cv::Vec3d>::WriteOrderedPointSet(filepath, ps);
}

// Load the PointSet from disk
auto Segmentation::getPointSet() const -> Segmentation::PointSet
{
    // Make sure there's an associated pointset file
    if (metadata_.get<std::string>("vcps").empty()) {
        throw std::runtime_error("segmentation has no pointset");
    }

    // Load the pointset
    auto filepath = path_ / metadata_.get<std::string>("vcps");
    return PointSetIO<cv::Vec3d>::ReadOrderedPointSet(filepath);
}