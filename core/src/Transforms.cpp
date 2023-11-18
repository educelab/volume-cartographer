#include "vc/core/types/Transforms.hpp"

#include <fstream>
#include <iomanip>

#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Json.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
namespace vc = volcart;
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

///////////////////////////////////////
///////////// Transform3D /////////////
///////////////////////////////////////

auto Transform3D::applyUnitVector(const cv::Vec3d& vector) const -> cv::Vec3d
{
    return cv::normalize(applyVector(vector));
}

auto Transform3D::applyPointAndNormal(
    const cv::Vec6d& ptN, bool normalize) const -> cv::Vec6d
{
    auto p = applyPoint({ptN[0], ptN[1], ptN[2]});
    auto n = (normalize) ? applyUnitVector({ptN[3], ptN[4], ptN[5]})
                         : applyVector({ptN[3], ptN[4], ptN[5]});
    return {p[0], p[1], p[2], n[0], n[1], n[2]};
}

void Transform3D::Save(
    const filesystem::path& path, const Transform3D::Pointer& transform)
{
    Metadata meta{{"type", "Transform3D"}};
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
    result->from_meta_(meta);

    return result;
}

auto Transform3D::invertible() const -> bool { return false; }

auto Transform3D::invert() const -> Transform3D::Pointer
{
    throw std::runtime_error(this->type() + " is not invertible");
}

///////////////////////////////////////////
///////////// AffineTransform /////////////
///////////////////////////////////////////

auto AffineTransform::New() -> AffineTransform::Pointer
{
    // Trick to allow classes with protected/private constructors to be
    // constructed with std::make_shared: https://stackoverflow.com/a/25069711
    struct EnableSharedHelper : public AffineTransform {
    };
    return std::make_shared<EnableSharedHelper>();
}

auto AffineTransform::applyPoint(const cv::Vec3d& point) const -> cv::Vec3d
{
    auto p = params_ * cv::Vec4d{point[0], point[1], point[2], 1.};
    return {p[0], p[1], p[2]};
}

auto AffineTransform::applyVector(const cv::Vec3d& vector) const -> cv::Vec3d
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

    // TODO: Can probably do this without using LU
    inverted->params_ = params_.inv();

    return inverted;
}

auto AffineTransform::type() const -> std::string { return "AffineTransform"; }

void AffineTransform::clear() { params_ = Parameters::eye(); }

auto AffineTransform::clone() const -> Transform3D::Pointer
{
    return std::make_shared<AffineTransform>(*this);
}

void AffineTransform::translate(double x, double y, double z)
{
    Parameters p = Parameters::eye();
    p(0, 3) = x;
    p(1, 3) = y;
    p(2, 3) = z;
    params_ = p * params_;
}

void AffineTransform::rotate(double theta, double x, double y, double z)
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
}

void AffineTransform::scale(double sx, double sy, double sz)
{
    const Parameters p{sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, 0, 0, 0, 1};
    params_ = p * params_;
}

void AffineTransform::scale(double s) { scale(s, s, s); }

auto operator<<(std::ostream& os, const AffineTransform& t) -> std::ostream&
{
    os << t.type() << "([";
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

//////////////////////////////////////////
///////////// ApplyTransform /////////////
//////////////////////////////////////////

auto vc::ApplyTransform(
    const ITKMesh::Pointer& mesh,
    const Transform3D::Pointer& transform,
    bool normalize) -> ITKMesh::Pointer
{
    // Generate a new mesh
    auto out = ITKMesh::New();

    // Copy and transform the vertices/normals
    for (auto pt = mesh->GetPoints()->Begin(); pt != mesh->GetPoints()->End();
         ++pt) {

        // Transform point
        auto p = pt->Value();
        auto tPt = transform->applyPoint({p[0], p[1], p[2]});
        p[0] = tPt[0];
        p[1] = tPt[1];
        p[2] = tPt[2];
        out->SetPoint(pt.Index(), p);

        // Transform vertex normal
        ITKPixel n;
        if (mesh->GetPointData(pt.Index(), &n)) {
            cv::Vec3d tNml;
            if (normalize) {
                tNml = transform->applyUnitVector({n[0], n[1], n[2]});
            } else {
                tNml = transform->applyVector({n[0], n[1], n[2]});
            }
            n[0] = tNml[0];
            n[1] = tNml[1];
            n[2] = tNml[2];
            out->SetPointData(pt.Index(), n);
        }
    }

    // Copy the faces
    DeepCopy(mesh, out, false, true);

    return out;
}

auto vc::ApplyTransform(
    const PerPixelMap& ppm,
    const Transform3D::Pointer& transform,
    bool normalize) -> PerPixelMap
{
    PerPixelMap output(ppm);

    for (auto m : output.getMappings()) {
        auto p = transform->applyPoint(m.pos);
        cv::Vec3d n;
        if (normalize) {
            n = transform->applyUnitVector(m.normal);
        } else {
            n = transform->applyVector(m.normal);
        }
        output(m.y, m.x) = {p[0], p[1], p[2], n[0], n[1], n[2]};
    }

    return output;
}

auto vc::ApplyTransform(
    const PerPixelMap::Pointer& ppm,
    const Transform3D::Pointer& transform,
    bool normalize) -> PerPixelMap::Pointer
{
    return PerPixelMap::New(ApplyTransform(*ppm, transform, normalize));
}
