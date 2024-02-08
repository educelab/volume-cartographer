#include "vc/core/landmarks/PlaneLandmark.hpp"

using namespace volcart;
using namespace volcart::landmarks;

PlaneLandmark::PlaneLandmark(const Identifier& uuid, const std::string& name)
    : VolumeLandmark(uuid, name, Type::Plane)
{
    update_meta_();
}

PlaneLandmark::PlaneLandmark(
    const Identifier& uuid,
    const std::string& name,
    const Point& center,
    const Point& normal)
    : VolumeLandmark(uuid, name, Type::Plane), center_{center}, normal_{normal}
{
    update_meta_();
}

auto PlaneLandmark::New(const Identifier& uuid, const std::string& name)
    -> PlaneLandmark::Pointer
{
    return std::make_shared<PlaneLandmark>(uuid, name);
}

auto PlaneLandmark::New(
    const Identifier& uuid,
    const std::string& name,
    const Point& center,
    const Point& normal) -> PlaneLandmark::Pointer
{
    return std::make_shared<PlaneLandmark>(uuid, name, center, normal);
}

void PlaneLandmark::setCenter(double x, double y, double z)
{
    center_[0] = x;
    center_[1] = y;
    center_[2] = z;
    update_meta_();
}

void PlaneLandmark::setCenter(const Point& values)
{
    center_ = values;
    update_meta_();
}

void PlaneLandmark::setNormal(double x, double y, double z)
{
    normal_[0] = x;
    normal_[1] = y;
    normal_[2] = z;
    update_meta_();
}

void PlaneLandmark::setNormal(const Point& values)
{
    normal_ = values;
    update_meta_();
}

auto PlaneLandmark::getCenter() const -> PlaneLandmark::Point
{
    return center_;
}

auto PlaneLandmark::getNormal() const -> PlaneLandmark::Point
{
    return normal_;
}

void PlaneLandmark::update_meta_()
{
    metadata_.set("center", center_);
    metadata_.set("normal", normal_);
}