#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"

namespace volcart
{

class Transform3D
{
public:
    virtual ~Transform3D() = default;

    /** Pointer type */
    using Pointer = std::shared_ptr<Transform3D>;

    void source(const std::string& src);
    [[nodiscard]] auto source() const -> std::string;
    void target(const std::string& tgt);
    [[nodiscard]] auto target() const -> std::string;

    virtual auto applyPoint(const cv::Vec3d& point) -> cv::Vec3d = 0;
    virtual auto applyVector(const cv::Vec3d& vector) -> cv::Vec3d = 0;
    auto applyUnitVector(const cv::Vec3d& vector) -> cv::Vec3d;
    auto applyPointAndNormal(
        const cv::Vec<double, 6>& ptN, bool normalize = true)
        -> cv::Vec<double, 6>;

    [[maybe_unused]] virtual auto invertible() const -> bool;
    virtual auto invert() const -> Pointer = 0;

    static void Save(const filesystem::path& path, const Pointer& transform);
    static auto Load(const filesystem::path& path) -> Pointer;

protected:
    Transform3D() = default;
    using Metadata = nlohmann::ordered_json;
    virtual void to_meta_(Metadata& meta) = 0;
    virtual void from_meta_(const Metadata& meta) = 0;

private:
    std::string src_;
    std::string tgt_;
};

class AffineTransform : public Transform3D
{
    using Parameters = cv::Matx<double, 4, 4>;

public:
    AffineTransform() = default;

    /** Pointer type */
    using Pointer = std::shared_ptr<AffineTransform>;

    /** Static New function for all constructors of T */
    template <typename... Args>
    static auto New(Args... args) -> Pointer
    {
        return std::make_shared<AffineTransform>(std::forward<Args>(args)...);
    }

    auto applyPoint(const cv::Vec3d& point) -> cv::Vec3d final;
    auto applyVector(const cv::Vec3d& vector) -> cv::Vec3d final;

    [[nodiscard]] auto invertible() const -> bool final;
    [[nodiscard]] auto invert() const -> Transform3D::Pointer final;

    [[nodiscard]] auto params() const -> Parameters;
    void params(const Parameters& params);

    auto translate(double x, double y, double z) -> AffineTransform&;

    template <typename Tp, int Cn>
    auto translate(const cv::Vec<Tp, Cn>& translation) -> AffineTransform&
    {
        static_assert(Cn >= 3, "cv::Vec must have >= 3 values");
        return translate(translation[0], translation[1], translation[2]);
    }

    auto rotate(double angle, double ax, double ay, double az) -> AffineTransform&;

    template <typename Tp, int Cn>
    auto rotate(double angle, cv::Vec<Tp, Cn> axis) -> AffineTransform&
    {
        static_assert(Cn >= 3, "cv::Vec must have >= 3 values");
        return rotate(angle, axis[0], axis[1], axis[2]);
    }

    auto scale(double sx, double sy, double sz) -> AffineTransform&;

    auto scale(double s) -> AffineTransform&;

private:
    Parameters params_{cv::Matx<double, 4, 4>::eye()};
    void to_meta_(Metadata& meta) final;
    void from_meta_(const Metadata& meta) final;
};

}  // namespace volcart

auto operator<<(std::ostream& os, const volcart::AffineTransform& t)
    -> std::ostream&;