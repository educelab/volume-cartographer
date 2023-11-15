#include "vc/core/types/Transforms.hpp"

#include "vc/core/util/Json.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;

void VolumeTransform::source(const Volume::Identifier& src) { src_ = src; }

auto VolumeTransform::source() const -> Volume::Identifier { return src_; }

void VolumeTransform::target(const Volume::Identifier& tgt) { tgt_ = tgt; }

auto VolumeTransform::target() const -> Volume::Identifier { return tgt_; }

auto VolumeTransform::applyPointAndNormal(const cv::Vec<double, 6>& ptN)
    -> cv::Vec<double, 6>
{
    auto p = applyPoint({ptN[0], ptN[1], ptN[2]});
    auto n = applyVector({ptN[3], ptN[4], ptN[5]});
    return {p[0], p[1], p[2], n[0], n[1], n[2]};
}

void VolumeTransform::Save(
    const filesystem::path& path, const VolumeTransform::Pointer& transform)
{
    Metadata meta;
    meta.set("type", "VolumeTransform");
    meta.set("source", transform->src_);
    meta.set("target", transform->tgt_);
    transform->to_meta_(meta);
    meta.save(path);
}

auto VolumeTransform::Load(const filesystem::path& path)
    -> VolumeTransform::Pointer
{
    const Metadata meta(path);
    if (meta.get<std::string>("type") != "VolumeTransform") {
        throw std::runtime_error("File not of type: VolumeTransform");
    }

    if (not meta.hasKey("class")) {
        throw std::runtime_error("Unspecified transform class");
    }

    auto tfmClass = meta.get<std::string>("class");
    VolumeTransform::Pointer result;
    if (tfmClass == "AffineTransform") {
        result = AffineTransform::New();
    } else {
        throw std::runtime_error("Unknown transform class: " + tfmClass);
    }
    result->from_meta_(meta);

    return result;
}
auto VolumeTransform::invertible() const -> bool { return false; }

auto AffineTransform::applyPoint(const cv::Vec3d& point) -> cv::Vec3d
{
    cv::Mat p = params_ * cv::Vec4d{point[0], point[1], point[2], 1.};
    return {p.at<double>(0), p.at<double>(1), p.at<double>(2)};
}

auto AffineTransform::applyVector(const cv::Vec3d& vector) -> cv::Vec3d
{
    cv::Mat p = params_ * cv::Vec4d{vector[0], vector[1], vector[2], 0.};
    return {p.at<double>(0), p.at<double>(1), p.at<double>(2)};
}

void AffineTransform::to_meta_(Metadata& meta)
{
    meta.set("class", "AffineTransform");
    meta.set("params", params_);
}

void AffineTransform::from_meta_(const Metadata& meta)
{
    params_ = meta.get<Parameters>("params");
}

auto AffineTransform::params() const -> AffineTransform::Parameters
{
    return params_;
}

void AffineTransform::params(const Parameters& params)
{
    if (params.rows != 4 or params.cols != 4 or params.channels() != 1) {
        auto r = std::to_string(params.rows);
        auto c = std::to_string(params.cols);
        auto cns = std::to_string(params.channels());
        throw std::invalid_argument(
            "Params have invalid shape: (" + r + "," + c + "," + cns +
            " Expected: (4,4,1)");
    }
    params_ = params.clone();
}

auto AffineTransform::invertible() const -> bool { return true; }

auto AffineTransform::invert() const -> VolumeTransform::Pointer
{
    using ROI = std::vector<cv::Range>;
    const ROI rotRoi{{0, 3}, {0, 3}};
    const ROI tRoi{{0, 3}, {3, 4}};
    auto inverted = AffineTransform::New();
    inverted->src_ = tgt_;
    inverted->tgt_ = src_;

    cv::copyTo(params_(rotRoi).t(), inverted->params_(rotRoi), {});
    inverted->params_(tRoi) = -params_(rotRoi).t() * params_(tRoi);

    return inverted;
}

auto AffineTransform::translate(double x, double y, double z)
    -> AffineTransform&
{
    Parameters p = Parameters::eye(4, 4);
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

    Parameters p = Parameters::eye(4, 4);
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
