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
    metadata_.set("vcano", std::string{});
    metadata_.set("volume", Volume::Identifier{});
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
Segmentation::PointSet Segmentation::getPointSet() const
{
    // Make sure there's an associated pointset file
    if (metadata_.get<std::string>("vcps").empty()) {
        throw std::runtime_error("segmentation has no pointset");
    }

    // Load the pointset
    auto filepath = path_ / metadata_.get<std::string>("vcps");
    return PointSetIO<cv::Vec3d>::ReadOrderedPointSet(filepath);
}

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Save the AnnotationSet to disk
void Segmentation::setAnnotationSet(const AnnotationSet& as)
{
    // Set a name into the metadata if we haven't set one already
    if (!metadata_.hasKey("vcano") || metadata_.get<std::string>("vcano").empty()) {
        metadata_.set("vcano", "pointset.vcano");
        metadata_.save();
    }

    auto long_to_double = overloaded{
        [](long l) { return (double)l; },
    };

    AnnotationSetRaw asRaw(as.width());
    for (size_t h = 0; h < as.height(); ++h) {
        std::vector<Segmentation::AnnotationRaw> anRowRaw;
        anRowRaw.reserve(as.width());
        for (size_t w = 0; w < as.width(); ++w) {
            AnnotationRaw anRaw;
            for (size_t i = 0; i < as[h * w].channels; ++i) {
                anRaw[i] = std::visit(long_to_double, as[h * w](i));
            }
            anRowRaw.push_back(anRaw);
        }
        asRaw.pushRow(anRowRaw);
    }

    // Write the annotation set to the segmentation file
    auto filepath = path_ / metadata_.get<std::string>("vcano");
    PointSetIO<Segmentation::AnnotationRaw>::WriteOrderedPointSet(filepath, asRaw);
}

// Load the AnnotationSet from disk
Segmentation::AnnotationSet Segmentation::getAnnotationSet() const
{
    // Check if there's an associated annotation set file
    if (metadata_.get<std::string>("vcano").empty()) {
        return Segmentation::AnnotationSet();
    }

    // Load the annotation set
    auto filepath = path_ / metadata_.get<std::string>("vcano");

    auto raw = PointSetIO<Segmentation::AnnotationRaw>::ReadOrderedPointSet(filepath);
    Segmentation::AnnotationSet as(raw.width());
    for (size_t h = 0; h < raw.height(); ++h) {
        std::vector<Segmentation::Annotation> an;
        an.reserve(raw.width());
        for (size_t w = 0; w < raw.width(); ++w) {
            an.emplace_back((long)raw[h * w][0], raw[h * w][1], raw[h * w][2]);
        }
        as.pushRow(an);
    }

    return as;
}