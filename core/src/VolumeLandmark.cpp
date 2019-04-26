#include "vc/core/landmarks/VolumeLandmark.hpp"
#include "vc/core/landmarks/PlaneLandmark.hpp"
#include "vc/core/landmarks/PointLandmark.hpp"
#include "vc/core/landmarks/PolylineLandmark.hpp"

using namespace volcart;
using namespace volcart::landmarks;

namespace fs = boost::filesystem;

VolumeLandmark::VolumeLandmark(
    const Identifier& uuid, const std::string& name, Type type)
{
    metadata_.set("uuid", uuid);
    metadata_.set("name", name);
    metadata_.set("type", "ldm");
    metadata_.set("ldmType", type);

    type_ = type;
}

VolumeLandmark::Identifier VolumeLandmark::id() const
{
    return metadata_.get<std::string>("uuid");
}

std::string VolumeLandmark::name() const
{
    return metadata_.get<std::string>("name");
}

Type VolumeLandmark::type() const { return type_; }

void VolumeLandmark::Write(const fs::path& path, const Pointer& ldm)
{
    ldm->metadata_.save(path);
}

VolumeLandmark::Pointer VolumeLandmark::Read(const fs::path& path)
{
    // Load metadata
    volcart::Metadata meta(path);
    if (meta.get<std::string>("type") != "ldm") {
        throw std::runtime_error("File not of type: ldm");
    }

    // Get basic info
    auto uuid = meta.get<std::string>("uuid");
    auto name = meta.get<std::string>("name");
    auto ldmType = meta.get<Type>("ldmType");

    // Construct the actual landmark type
    VolumeLandmark::Pointer result;
    switch (ldmType) {
        // Handle PointLandmark
        case Type::Point: {
            auto position = meta.get<Point>("position");
            auto tmp = PointLandmark::New(uuid, name, position);
            result = std::static_pointer_cast<VolumeLandmark>(tmp);
            break;
        }

        // Handle PlaneLandmark
        case Type::Plane: {
            auto center = meta.get<Point>("center");
            auto normal = meta.get<Point>("normal");
            auto tmp = PlaneLandmark::New(uuid, name, center, normal);
            result = std::static_pointer_cast<VolumeLandmark>(tmp);
            break;
        }

        // Handle PolylineLandmark
        case Type::Polyline: {
            auto pts = meta.get<std::vector<Point>>("position");
            auto tmp = PolylineLandmark::New(uuid, name, pts);
            result = std::static_pointer_cast<VolumeLandmark>(tmp);
            break;
        }
    }

    // Assign the metadata to result landmark
    result->metadata_ = meta;

    return result;
}