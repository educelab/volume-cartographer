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
        [](double d) { return d; },
    };

    // Convert from the long and double variant to only double values for storing
    AnnotationSetRaw asRaw(as.width());
    for (std::size_t h = 0; h < as.height(); ++h) {
        std::vector<Segmentation::AnnotationRaw> asRowRaw(as.width());
        for (std::size_t w = 0; w < as.width(); ++w) {
            AnnotationRaw anRaw;
            for (std::size_t i = 0; i < as[h * as.width() + w].channels; ++i) {
                anRaw[i] = std::visit(long_to_double, as[h * as.width() + w](i));
            }
            asRowRaw[w] = anRaw;
        }
        asRaw.pushRow(asRowRaw);
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
    try {
        auto raw = PointSetIO<Segmentation::AnnotationRaw>::ReadOrderedPointSet(filepath);

        // Convert from raw (only double values) to long and double variant
        Segmentation::AnnotationSet as(raw.width());
        for (std::size_t h = 0; h < raw.height(); ++h) {
            std::vector<Segmentation::Annotation> asRow(raw.width());
            for (std::size_t w = 0; w < raw.width(); ++w) {
                asRow[w] = Annotation((long)raw[h * raw.width() + w][0], (long)raw[h * raw.width() + w][1], raw[h * raw.width() + w][2], raw[h * raw.width() + w][3]);
            }
            as.pushRow(asRow);
        }

        return as;

    } catch (IOException) {
        return Segmentation::AnnotationSet();
    }
}