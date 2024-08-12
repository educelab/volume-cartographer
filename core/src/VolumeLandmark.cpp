#include "vc/core/landmarks/VolumeLandmark.hpp"
#include "vc/core/landmarks/PlaneLandmark.hpp"
#include "vc/core/landmarks/PointLandmark.hpp"
#include "vc/core/landmarks/PolylineLandmark.hpp"

using namespace volcart;
using namespace volcart::landmarks;

namespace fs = volcart::filesystem;

VolumeLandmark::VolumeLandmark(
    const Identifier& uuid, const std::string& name, Type type)
{
    metadata_.set("uuid", uuid);
    metadata_.set("name", name);
    metadata_.set("type", "ldm");
    metadata_.set("ldmType", type);

    type_ = type;
}

auto VolumeLandmark::id() const -> VolumeLandmark::Identifier
{
    return metadata_.get<std::string>("uuid").value();
}

auto VolumeLandmark::name() const -> std::string
{
    return metadata_.get<std::string>("name").value();
}

auto VolumeLandmark::type() const -> Type { return type_; }

void VolumeLandmark::Write(const fs::path& path, const Pointer& ldm)
{
    ldm->metadata_.save(path);
}

auto VolumeLandmark::Read(const fs::path& path) -> VolumeLandmark::Pointer
{
    // Load metadata
    volcart::Metadata meta(path);
    if (meta.get<std::string>("type") != "ldm") {
        throw std::runtime_error("File not of type: ldm");
    }

    // Get basic info
    auto uuid = meta.get<std::string>("uuid").value();
    auto name = meta.get<std::string>("name").value();
    auto ldmType = meta.get<Type>("ldmType").value();

    // Construct the actual landmark type
    VolumeLandmark::Pointer result;
    switch (ldmType) {
        // Handle PointLandmark
        case Type::Point: {
            const auto position = meta.get<Point>("position").value();
            const auto tmp = PointLandmark::New(uuid, name, position);
            result = std::static_pointer_cast<VolumeLandmark>(tmp);
            break;
        }

        // Handle PlaneLandmark
        case Type::Plane: {
            const auto center = meta.get<Point>("center").value();
            const auto normal = meta.get<Point>("normal").value();
            const auto tmp = PlaneLandmark::New(uuid, name, center, normal);
            result = std::static_pointer_cast<VolumeLandmark>(tmp);
            break;
        }

        // Handle PolylineLandmark
        case Type::Polyline: {
            const auto pts = meta.get<std::vector<Point>>("position").value();
            const auto tmp = PolylineLandmark::New(uuid, name, pts);
            result = std::static_pointer_cast<VolumeLandmark>(tmp);
            break;
        }
    }

    // Assign the metadata to result landmark
    result->metadata_ = meta;

    return result;
}