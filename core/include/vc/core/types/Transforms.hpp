#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{

class VolumeTransform
{
public:
    virtual ~VolumeTransform() = default;

    /** Pointer type */
    using Pointer = std::shared_ptr<VolumeTransform>;

    void source(const Volume::Identifier& src);
    [[nodiscard]] auto source() const -> Volume::Identifier;
    void target(const Volume::Identifier& tgt);
    [[nodiscard]] auto target() const -> Volume::Identifier;

    virtual auto applyPoint(const cv::Vec3d& point) -> cv::Vec3d = 0;
    virtual auto applyVector(const cv::Vec3d& vector) -> cv::Vec3d = 0;
    auto applyPointAndNormal(const cv::Vec<double, 6>& ptN)
        -> cv::Vec<double, 6>;

    [[maybe_unused]] virtual auto invertible() const -> bool;
    virtual auto invert() const -> Pointer = 0;

    static void Save(const filesystem::path& path, const Pointer& transform);
    static auto Load(const filesystem::path& path) -> Pointer;

protected:
    VolumeTransform() = default;
    virtual void to_meta_(Metadata& meta) = 0;
    virtual void from_meta_(const Metadata& meta) = 0;
    Volume::Identifier src_;
    Volume::Identifier tgt_;
};

class AffineTransform : public VolumeTransform
{
    using Parameters = cv::Mat_<double>;

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
    [[nodiscard]] auto invert() const -> VolumeTransform::Pointer final;

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

private:
    Parameters params_{cv::Mat_<double>::eye(4, 4)};
    void to_meta_(Metadata& meta) final;
    void from_meta_(const Metadata& meta) final;
};

}  // namespace volcart
