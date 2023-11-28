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
    o << m << "\n";
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

void Transform3D::source(const std::string& src) { src_ = src; }

auto Transform3D::source() const -> std::string { return src_; }

void Transform3D::target(const std::string& tgt) { tgt_ = tgt; }

auto Transform3D::target() const -> std::string { return tgt_; }

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

void Transform3D::clear()
{
    src_.clear();
    tgt_.clear();
    reset();
}

auto Transform3D::Compose(const Pointer& lhs, const Pointer& rhs)
    -> std::pair<Pointer, Pointer>
{
    // Return the inputs if either are not composable
    if (not lhs->composable() or not rhs->composable()) {
        return {lhs, rhs};
    }

    // Defer to the transform for composition
    return {lhs->compose_(rhs), nullptr};
}

auto Transform3D::compose_(const Pointer& rhs) const -> Transform3D::Pointer
{
    return nullptr;
}

auto Transform3D::Serialize(const Pointer& transform) -> Metadata
{
    Metadata meta{
        {"type", "Transform3D"},
        {"source", transform->src_},
        {"target", transform->tgt_},
        {"transform-type", transform->type()}};
    transform->to_meta_(meta);
    return meta;
}

auto Transform3D::Deserialize(const Metadata& meta) -> Pointer
{
    if (meta["type"].get<std::string>() != "Transform3D") {
        throw std::runtime_error("File not of type: Transform3D");
    }

    if (not meta.contains("transform-type")) {
        throw std::runtime_error("Unspecified transform type");
    }

    auto tfmType = meta["transform-type"].get<std::string>();
    Pointer result;
    if (tfmType == AffineTransform::TYPE) {
        result = AffineTransform::New();
    } else if (tfmType == IdentityTransform::TYPE) {
        result = IdentityTransform::New();
    } else if (tfmType == CompositeTransform::TYPE) {
        result = CompositeTransform::New();
    } else {
        throw std::runtime_error("Unknown transform type: " + tfmType);
    }
    result->src_ = meta["source"].get<std::string>();
    result->tgt_ = meta["target"].get<std::string>();
    result->from_meta_(meta);

    return result;
}

void Transform3D::Save(const filesystem::path& path, const Pointer& transform)
{
    ::WriteMetadata(path, Serialize(transform));
}

auto Transform3D::Load(const filesystem::path& path) -> Pointer
{
    auto meta = ::LoadMetadata(path);
    return Deserialize(meta);
}

auto Transform3D::invertible() const -> bool { return false; }

auto Transform3D::invert() const -> Pointer
{
    throw std::runtime_error(std::string(this->type()) + " is not invertible");
}

auto Transform3D::composable() const -> bool { return false; }

///////////////////////////////////////////
///////////// AffineTransform /////////////
///////////////////////////////////////////

auto AffineTransform::New() -> Pointer
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

void AffineTransform::to_meta_(Metadata& meta) { meta["params"] = params_; }

void AffineTransform::from_meta_(const Metadata& meta)
{
    params_ = meta["params"].get<Parameters>();
}

auto AffineTransform::params() const -> Parameters { return params_; }

void AffineTransform::params(const Parameters& params) { params_ = params; }

auto AffineTransform::invertible() const -> bool { return true; }

auto AffineTransform::invert() const -> Transform3D::Pointer
{
    auto inverted = AffineTransform::New();
    inverted->source(target());
    inverted->target(source());

    // TODO: Can probably do this without using LU
    inverted->params_ = params_.inv();

    return inverted;
}

auto AffineTransform::composable() const -> bool { return true; }

auto AffineTransform::compose_(const Transform3D::Pointer& rhs) const
    -> Transform3D::Pointer
{
    // Get a copy of self
    auto res = AffineTransform::New();

    // Copy Transform3D parameters
    res->source(source());
    res->target(rhs->target());

    // If IdentityTransform
    if (rhs->type() == IdentityTransform::TYPE) {
        // No-op
    } else if (rhs->type() == AffineTransform::TYPE) {
        // Compose the parameters
        auto affRhs = std::dynamic_pointer_cast<AffineTransform>(rhs);
        if (not affRhs) {
            throw std::invalid_argument("rhs argument is not AffineTransform");
        }
        res->params_ = affRhs->params_ * params_;
    } else {
        throw std::invalid_argument(
            "Unsupported transform type: " + std::string(rhs->type()));
    }

    return res;
}

auto AffineTransform::type() const -> std::string_view { return TYPE; }

void AffineTransform::reset() { params_ = Parameters::eye(); }

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

/////////////////////////////////////////////
///////////// IdentityTransform /////////////
/////////////////////////////////////////////

auto IdentityTransform::New() -> Pointer
{
    struct EnableSharedHelper : public IdentityTransform {
    };
    return std::make_shared<EnableSharedHelper>();
}

auto IdentityTransform::type() const -> std::string_view { return TYPE; }

auto IdentityTransform::clone() const -> Transform3D::Pointer
{
    return std::make_shared<IdentityTransform>(*this);
}

auto IdentityTransform::invertible() const -> bool { return true; }

auto IdentityTransform::invert() const -> Transform3D::Pointer
{
    auto ret = New();
    ret->source(target());
    ret->target(source());
    return ret;
}

auto IdentityTransform::composable() const -> bool { return true; }

auto IdentityTransform::compose_(const Transform3D::Pointer& rhs) const
    -> Transform3D::Pointer
{
    auto res = rhs->clone();
    res->source(source());

    return res;
}

void IdentityTransform::reset() {}

auto IdentityTransform::applyPoint(const cv::Vec3d& point) const -> cv::Vec3d
{
    return point;
}

auto IdentityTransform::applyVector(const cv::Vec3d& vector) const -> cv::Vec3d
{
    return vector;
}

void IdentityTransform::to_meta_(Metadata& meta) {}

void IdentityTransform::from_meta_(const Metadata& meta) {}

//////////////////////////////////////////////
///////////// CompositeTransform /////////////
//////////////////////////////////////////////

auto CompositeTransform::New() -> Pointer
{
    struct EnableSharedHelper : public CompositeTransform {
    };
    return std::make_shared<EnableSharedHelper>();
}

auto CompositeTransform::type() const -> std::string_view { return TYPE; }

auto CompositeTransform::clone() const -> Transform3D::Pointer
{
    return std::make_shared<CompositeTransform>(*this);
}

void CompositeTransform::reset() { tfms_.clear(); }

auto CompositeTransform::applyPoint(const cv::Vec3d& point) const -> cv::Vec3d
{
    auto pt = point;
    for (const auto& t : tfms_) {
        pt = t->applyPoint(pt);
    }
    return pt;
}

auto CompositeTransform::applyVector(const cv::Vec3d& vector) const -> cv::Vec3d
{
    auto vec = vector;
    for (const auto& t : tfms_) {
        vec = t->applyVector(vec);
    }
    return vector;
}

void CompositeTransform::push_back(const Transform3D::Pointer& t)
{
    // Easy case: Not a composite transform
    if (t->type() != CompositeTransform::TYPE) {
        tfms_.push_back(t->clone());
        return;
    }

    // Hard case: Composite transforms should be expanded
    std::list<Transform3D::Pointer> queue{t};
    while (not queue.empty()) {
        // Get the next transform
        auto tfm = queue.front();
        queue.pop_front();

        // If a composite transform, push its stack to the front of the queue
        if (t->type() == CompositeTransform::TYPE) {
            auto cmp = std::dynamic_pointer_cast<CompositeTransform>(t);
            queue.insert(queue.begin(), cmp->tfms_.begin(), cmp->tfms_.end());
        }
        // Otherwise, add this tfm to the queue
        else {
            tfms_.push_back(t->clone());
        }
    }
}

auto CompositeTransform::size() const noexcept -> std::size_t
{
    return tfms_.size();
}

void CompositeTransform::simplify()
{
    std::vector<Transform3D::Pointer> newTfms;
    Transform3D::Pointer lhs;
    Transform3D::Pointer rhs;
    for (const auto& tfm : tfms_) {
        // Set our first LHS
        if (not lhs) {
            lhs = tfm;
            continue;
        }

        // Try compose
        std::tie(lhs, rhs) = Compose(lhs, tfm);

        // if rhs, then couldn't compose
        // push lhs onto new list. rhs is new lhs.
        if (rhs) {
            newTfms.push_back(lhs);
            lhs = rhs;
        }
    }
    // push the final lhs
    newTfms.push_back(lhs);

    // replace the current list
    tfms_ = newTfms;
}

void CompositeTransform::to_meta_(Metadata& meta)
{
    Metadata stack;
    for (const auto& tfm : tfms_) {
        stack.push_back(Serialize(tfm));
    }
    meta["transform-stack"] = stack;
}

void CompositeTransform::from_meta_(const Metadata& meta)
{
    tfms_.clear();
    for (const auto& m : meta["transform-stack"]) {
        push_back(Deserialize(m));
    }
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