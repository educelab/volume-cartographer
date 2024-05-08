#include "vc/core/landmarks/PolylineLandmark.hpp"

using namespace volcart;
using namespace volcart::landmarks;

PolylineLandmark::PolylineLandmark(
    const Identifier& uuid, const std::string& name)
    : VolumeLandmark(uuid, name, Type::Polyline)
{
    update_meta_();
}

PolylineLandmark::PolylineLandmark(
    const Identifier& uuid, const std::string& name, Polyline poly)
    : VolumeLandmark(uuid, name, Type::Polyline), poly_{std::move(poly)}
{
    update_meta_();
}

auto PolylineLandmark::New(const Identifier& uuid, const std::string& name)
    -> PolylineLandmark::Pointer
{
    return std::make_shared<PolylineLandmark>(uuid, name);
}

auto PolylineLandmark::New(
    const Identifier& uuid, const std::string& name, const Polyline& poly)
    -> PolylineLandmark::Pointer
{
    return std::make_shared<PolylineLandmark>(uuid, name, poly);
}

void PolylineLandmark::setPolyline(Polyline p)
{
    poly_ = std::move(p);
    update_meta_();
}

auto PolylineLandmark::getPolyline() const -> PolylineLandmark::Polyline
{
    return poly_;
}

void PolylineLandmark::addPoint(const Point& pt)
{
    poly_.push_back(pt);
    update_meta_();
}

void PolylineLandmark::addPoint(double x, double y, double z)
{
    poly_.emplace_back(x, y, z);
    update_meta_();
}

void PolylineLandmark::update_meta_() { metadata_.set("vertices", poly_); }