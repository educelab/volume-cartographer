#include "vc/core/types/Segmentation.hpp"

#include "vc/core/io/PointSetIO.hpp"

using namespace volcart;

namespace fs = filesystem;

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
    metadata_.set("vcps", nlohmann::json::value_t::null);
    metadata_.set("vcano", nlohmann::json::value_t::null);
    metadata_.set("volume", nlohmann::json::value_t::null);
    metadata_.save();
}

// Load a Segmentation from disk, return a pointer
auto Segmentation::New(const fs::path& path) -> Pointer
{
    return std::make_shared<Segmentation>(path);
}

// Make a new segmentation on disk, return a pointer
auto Segmentation::New(
    const fs::path& path,
    const std::string& uuid,
    const std::string& name) -> Pointer
{
    return std::make_shared<Segmentation>(path, uuid, name);
}

auto Segmentation::hasPointSet() const -> bool
{
    if (not metadata_.hasKey("vcps")) {
        return false;
    }
    const auto res = metadata_.get<std::string>("vcps");
    return res.has_value() and not res.value().empty();
}

// Save the PointSet to disk
void Segmentation::setPointSet(const PointSet& ps)
{
    // Set a name into the metadata if we haven't set one already
    if (not hasPointSet()) {
        metadata_.set("vcps", "pointset.vcps");
        metadata_.save();
    }

    // Write the pointset to the segmentation file
    const auto path = path_ / metadata_.get<std::string>("vcps").value();
    PointSetIO<cv::Vec3d>::WriteOrderedPointSet(path, ps);
}

// Load the PointSet from disk
auto Segmentation::getPointSet() const -> PointSet
{
    // Make sure there's an associated pointset file
    if (not hasPointSet()) {
        throw std::runtime_error("segmentation has no pointset");
    }

    // Load the pointset
    const auto path = path_ / metadata_.get<std::string>("vcps").value();
    return PointSetIO<cv::Vec3d>::ReadOrderedPointSet(path);
}

auto Segmentation::hasAnnotationSet() const -> bool
{
    if (not metadata_.hasKey("vcano")) {
        return false;
    }
    const auto res = metadata_.get<std::string>("vcano");
    return res.has_value() and not res.value().empty();
}

// Save the AnnotationSet to disk
void Segmentation::setAnnotationSet(const AnnotationSet& as)
{
    // Set a name into the metadata if we haven't set one already
    if (not hasAnnotationSet()) {
        metadata_.set("vcano", "pointset.vcano");
        metadata_.save();
    }

    // Write the annotation set to the segmentation file
    const auto p = path_ / metadata_.get<std::string>("vcano").value();
    WriteAnnotationSet(p, as);
}

// Load the AnnotationSet from disk
auto Segmentation::getAnnotationSet() const -> AnnotationSet
{
    // Check if there's an associated annotation set file
    if (not hasAnnotationSet()) {
        throw std::runtime_error("Segmentation has no annotation set");
    }

    // Load the annotation set
    const auto p = path_ / metadata_.get<std::string>("vcano").value();
    return ReadAnnotationSet(p);
}

auto Segmentation::hasVolumeID() const -> bool
{
    if (not metadata_.hasKey("volume")) {
        return false;
    }
    const auto res = metadata_.get<std::string>("volume");
    return res.has_value() and not res.value().empty();
}

auto Segmentation::getVolumeID() const -> Identifier
{
    if (not hasVolumeID()) {
        throw std::runtime_error("segmentation has no volume ID");
    }
    return metadata_.get<Identifier>("volume").value();
}

void Segmentation::setVolumeID(const Identifier& id)
{
    metadata_.set<std::string>("volume", id);
    metadata_.save();
}