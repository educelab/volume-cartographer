#include "vc/core/types/Transforms.hpp"

#include <fstream>
#include <iomanip>

#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Json.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;

namespace
{
void WriteMetadata(const fs::path& path, const nlohmann::ordered_json& m)
{
    std::ofstream o{path};
    o << m << std::endl;
}

auto LoadMetadata(const fs::path& path) -> nlohmann::ordered_json
{
    nlohmann::ordered_json m;
    std::ifstream i{path};
    i >> m;
    return m;
}
}  // namespace

void Transform3D::source(const std::string& src) { src_ = src; }

auto Transform3D::source() const -> std::string { return src_; }

void Transform3D::target(const std::string& tgt) { tgt_ = tgt; }

auto Transform3D::target() const -> std::string { return tgt_; }

auto Transform3D::applyUnitVector(const cv::Vec3d& vector) -> cv::Vec3d
{
    return cv::normalize(applyVector(vector));
}

auto Transform3D::applyPointAndNormal(
    const cv::Vec<double, 6>& ptN, bool normalize) -> cv::Vec<double, 6>
{
    auto p = applyPoint({ptN[0], ptN[1], ptN[2]});
    auto n = (normalize) ? applyUnitVector({ptN[3], ptN[4], ptN[5]})
                         : applyVector({ptN[3], ptN[4], ptN[5]});
    return {p[0], p[1], p[2], n[0], n[1], n[2]};
}

void Transform3D::Save(
    const filesystem::path& path, const Transform3D::Pointer& transform)
{
    Metadata meta{
        {"type", "Transform3D"},
        {"source", transform->src_},
        {"target", transform->tgt_}};
    transform->to_meta_(meta);
    ::WriteMetadata(path, meta);
}

auto Transform3D::Load(const filesystem::path& path) -> Transform3D::Pointer
{
    auto meta = ::LoadMetadata(path);
    if (meta["type"].get<std::string>() != "Transform3D") {
        throw std::runtime_error("File not of type: Transform3D");
    }

    if (not meta.contains("transform-type")) {
        throw std::runtime_error("Unspecified transform type");
    }

    auto tfmType = meta["transform-type"].get<std::string>();
    Transform3D::Pointer result;
    if (tfmType == "AffineTransform") {
        result = AffineTransform::New();
    } else {
        throw std::runtime_error("Unknown transform type: " + tfmType);
    }
    result->src_ = meta["source"].get<std::string>();
    result->tgt_ = meta["target"].get<std::string>();
    result->from_meta_(meta);

    return result;
}
auto Transform3D::invertible() const -> bool { return false; }

auto AffineTransform::applyPoint(const cv::Vec3d& point) -> cv::Vec3d
{
    auto p = params_ * cv::Vec4d{point[0], point[1], point[2], 1.};
    return {p[0], p[1], p[2]};
}

auto AffineTransform::applyVector(const cv::Vec3d& vector) -> cv::Vec3d
{
    auto p = params_ * cv::Vec4d{vector[0], vector[1], vector[2], 0.};
    return {p[0], p[1], p[2]};
}

void AffineTransform::to_meta_(Metadata& meta)
{
    meta["transform-type"] = "AffineTransform";
    meta["params"] = params_;
}

void AffineTransform::from_meta_(const Metadata& meta)
{
    params_ = meta["params"].get<Parameters>();
}

auto AffineTransform::params() const -> AffineTransform::Parameters
{
    return params_;
}

void AffineTransform::params(const Parameters& params) { params_ = params; }

auto AffineTransform::invertible() const -> bool { return true; }

auto AffineTransform::invert() const -> Transform3D::Pointer
{
    auto inverted = AffineTransform::New();
    inverted->source(target());
    inverted->target(source());

    // Invert scale
    cv::Matx33d tlInv;
    for (auto colIdx : range(3)) {
        auto col = params_.col(colIdx);
        col /= cv::norm(col);
        tlInv(0, colIdx) = col(0, 0);
        tlInv(1, colIdx) = col(1, 0);
        tlInv(2, colIdx) = col(2, 0);
    }

    // Invert rotation
    tlInv = tlInv.t();

    // Invert translation
    auto transInv = -tlInv * params_.get_minor<3, 1>(0, 3);

    // Copy top-left
    for (auto [y, x] : range2D(3, 3)) {
        inverted->params_(y, x) = tlInv(y, x);
    }

    // Copy translation
    inverted->params_(0, 3) = transInv(0, 0);
    inverted->params_(1, 3) = transInv(1, 0);
    inverted->params_(2, 3) = transInv(2, 0);

    return inverted;
}

auto AffineTransform::translate(double x, double y, double z)
    -> AffineTransform&
{
    Parameters p = Parameters::eye();
    p(0, 3) = x;
    p(1, 3) = y;
    p(2, 3) = z;
    params_ = p * params_;
    return *this;
}

auto AffineTransform::rotate(double theta, double x, double y, double z)
    -> AffineTransform&
{
    static constexpr double PI{
        3.141592653589793238462643383279502884198716939937510582097164L};

    auto norm = std::sqrt(x * x + y * y + z * z);
    x /= norm;
    y /= norm;
    z /= norm;

    auto radians = theta * PI / 180.;
    auto s = std::sin(radians);
    auto c = std::cos(radians);

    Parameters p = Parameters::eye();
    p(0, 0) = x * x * (1 - c) + c;
    p(0, 1) = x * y * (1 - c) - z * s;
    p(0, 2) = x * z * (1 - c) + y * s;
    p(1, 0) = y * x * (1 - c) + z * s;
    p(1, 1) = y * y * (1 - c) + c;
    p(1, 2) = y * z * (1 - c) - x * s;
    p(2, 0) = x * z * (1 - c) - y * s;
    p(2, 1) = y * z * (1 - c) + x * s;
    p(2, 2) = z * z * (1 - c) + c;
    params_ = p * params_;
    return *this;
}

auto AffineTransform::scale(double sx, double sy, double sz) -> AffineTransform&
{
    const Parameters p{sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, 0, 0, 0, 1};
    params_ = p * params_;
    return *this;
}

auto AffineTransform::scale(double s) -> AffineTransform&
{
    return scale(s, s, s);
}

auto operator<<(std::ostream& os, const AffineTransform& t) -> std::ostream&
{
    os << "AffineTransform([";
    for (auto [y, x] : range2D(4, 4)) {
        (x == 0) ? os << "[" : os << ", ";
        os << t.params()(y, x);
        if (x == 3) {
            if (y == 3) {
                os << "]";
            } else {
                os << "], ";
            }
        }
    }
    os << "])";
    return os;
}