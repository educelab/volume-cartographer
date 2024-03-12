#include "vc/core/landmarks/PointLandmark.hpp"

using namespace volcart;
using namespace volcart::landmarks;

PointLandmark::PointLandmark(const Identifier& uuid, const std::string& name)
    : VolumeLandmark(uuid, name, Type::Point)
{
    update_meta_();
}

PointLandmark::PointLandmark(
    const Identifier& uuid, const std::string& name, const Point& pos)
    : VolumeLandmark(uuid, name, Type::Point), position_{pos}
{
    update_meta_();
}

auto PointLandmark::New(const Identifier& uuid, const std::string& name)
    -> PointLandmark::Pointer
{
    return std::make_shared<PointLandmark>(uuid, name);
}

auto PointLandmark::New(
    const Identifier& uuid, const std::string& name, const cv::Vec3d& position)
    -> PointLandmark::Pointer
{
    return std::make_shared<PointLandmark>(uuid, name, position);
}

void PointLandmark::setPosition(const cv::Vec3d& pos)
{
    position_ = pos;
    update_meta_();
}

void PointLandmark::setPosition(double x, double y, double z)
{
    position_[0] = x;
    position_[1] = y;
    position_[2] = z;
    update_meta_();
}

auto PointLandmark::getPosition() const -> PointLandmark::Point
{
    return position_;
}

void PointLandmark::update_meta_() { metadata_.set("position", position_); }