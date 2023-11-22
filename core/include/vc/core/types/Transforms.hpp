#pragma once

/** @file */

#include <iostream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/PerPixelMap.hpp"

namespace volcart
{

/**
 * @brief Base class for 3D transforms
 *
 * Provides a common interface for all 3D transforms. Transform implementations
 * should derive from this class and be careful to implement all virtual
 * functions. Because Transform3D objects are polymorphic in nature, they must
 * be constructed on the heap using a New() static function or copied using the
 * clone() member function.
 *
 * Transforms can be applied from either the base or derived class.
 *
 * @code{.cpp}
 * // Construct and apply a transform
 * auto tfm = AffineTransform::New();
 * tfm.scale(10);
 * tfm.translate(0, 10, 0);
 * auto fromDerived = tfm->applyPoint({1, 1, 1});
 *
 * // Convert to base class and apply transform
 * Transform::Pointer baseTfm = tfm;
 * auto fromBase = baseTfm->applyPoint({1, 1, 1});
 *
 * if(fromDerived == fromBase) {
 *   std::cout << "Matches." << std::endl;
 * }
 * @endcode
 */
class Transform3D
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<Transform3D>;

    /** Transform identifier type (for VolumePkg) */
    using Identifier = std::string;

    /** Default constructor */
    virtual ~Transform3D() = default;
    /** Disallow move construction */
    Transform3D(Transform3D&&) = delete;
    /** Disallow move assignment */
    auto operator=(Transform3D&& other) -> Transform3D& = delete;

    /** @brief Return a string representation of the transform type */
    [[nodiscard]] virtual auto type() const -> std::string = 0;
    /** @brief Clone the transform */
    [[nodiscard]] virtual auto clone() const -> Pointer = 0;

    /** @brief Return whether the underlying transform is invertible */
    [[nodiscard]] virtual auto invertible() const -> bool;
    /** @brief Return the inverted transform */
    [[nodiscard]] virtual auto invert() const -> Pointer;

    /**
     * @brief Reset the transform parameters
     *
     * Resets all parameters controlled by the derived transform. Does not
     * reset base transform properties like source and target.
     *
     * @see Transform3D::clear()
     */
    virtual void reset() = 0;
    /** @brief Clears all parameters and properties of the transform */
    virtual void clear();

    /**
     * @brief Set the identifier for the source space
     *
     * This is a convenience feature for tracking the input and output spaces
     * for the transform. This is usually a Volume::Identifier.
     */
    void source(const std::string& src);
    /**
     * @brief Get the source space identifier
     *
     * @copydetails source(const std::string&)
     */
    [[nodiscard]] auto source() const -> std::string;
    /**
     * @brief Set the identifier for the target space
     *
     * @copydetails source(const std::string&)
     */
    void target(const std::string& tgt);
    /**
     * @brief Set the identifier for the target space
     *
     * @copydetails source(const std::string&)
     */
    [[nodiscard]] auto target() const -> std::string;

    /** @brief Transform a 3D point */
    [[nodiscard]] virtual auto applyPoint(const cv::Vec3d& point) const
        -> cv::Vec3d = 0;
    /** @brief Transform a 3D direction vector */
    [[nodiscard]] virtual auto applyVector(const cv::Vec3d& vector) const
        -> cv::Vec3d = 0;
    /**
     * @brief Transform a 3D direction unit vector
     *
     * This is a convenience function for vectors which should be normalized
     * after transformation (e.g. surface normals).
     */
    [[nodiscard]] auto applyUnitVector(const cv::Vec3d& vector) const
        -> cv::Vec3d;
    /**
     * @brief Transform a 3D point and surface normal stored in a cv::Vec6d
     *
     * This is a convenience function for transforming values stored in a
     * PerPixelMap.
     *
     * @param ptN The point and normal to be transformed.
     * @param normalize If true (default), the normal component of the output
     * (the last 3 elements in the output) will be normalized.
     */
    [[nodiscard]] auto applyPointAndNormal(
        const cv::Vec6d& ptN, bool normalize = true) const -> cv::Vec6d;

    /** @brief Save a transform to a JSON file */
    static void Save(const filesystem::path& path, const Pointer& transform);
    /** @brief Load a transform from a JSON file */
    static auto Load(const filesystem::path& path) -> Pointer;

protected:
    /** Only derived classes can construct */
    Transform3D() = default;
    /** Only derived classes can copy */
    Transform3D(const Transform3D&) = default;
    /** Only derived classes can copy */
    auto operator=(const Transform3D& other) -> Transform3D& = default;

    /** On-disk metadata type */
    using Metadata = nlohmann::ordered_json;
    /** Serialize the derived class parameters */
    virtual void to_meta_(Metadata& meta) = 0;
    /** Deserialize the derived class parameters */
    virtual void from_meta_(const Metadata& meta) = 0;

private:
    /** Source space identifier */
    std::string src_;
    /** Target space identifier */
    std::string tgt_;
};

/**
 * @brief 3D affine transform
 *
 * Initializes to an identity. Modify the transform by calling the translate(),
 * rotate(), and scale() functions. Each call precomposes the function's
 * transform with the stored affine transform. For example, the following
 * transform will scale, rotate, and translate the 3D point, in that order:
 *
 * @code
 * auto tfm = AffineTransform::New();
 * // scale by 5
 * tfm->scale(5);
 * // rotate 90 degrees around z-axis
 * tfm->rotate(90, 0, 0, 1);
 * // translate along y
 * tfm->translate(0, 10, 0);
 * // apply to a point
 * auto pt = tfm->applyPoint({0, 1, 0}); // {-5, 10, 0}
 * @endcode
 */
class AffineTransform : public Transform3D
{
public:
    /** Parameters type: 4x4 matrix */
    using Parameters = cv::Matx<double, 4, 4>;

    /** Pointer type */
    using Pointer = std::shared_ptr<AffineTransform>;

    /** @brief Create a new AffineTransform */
    static auto New() -> Pointer;

    /** @copydoc Transform3D::type() */
    [[nodiscard]] auto type() const -> std::string final;
    /** @copydoc Transform3D::clone() */
    [[nodiscard]] auto clone() const -> Transform3D::Pointer final;
    /** @copydoc Transform3D::invertible() */
    [[nodiscard]] auto invertible() const -> bool final;
    /** @copydoc Transform3D::invert() */
    [[nodiscard]] auto invert() const -> Transform3D::Pointer final;
    /** @copydoc Transform3D::reset() */
    void reset() final;
    /** @copydoc Transform3D::clear() */
    void clear() final;

    /** @copydoc Transform3D::applyPoint() */
    [[nodiscard]] auto applyPoint(const cv::Vec3d& point) const
        -> cv::Vec3d final;
    /** @copydoc Transform3D::applyVector() */
    [[nodiscard]] auto applyVector(const cv::Vec3d& vector) const
        -> cv::Vec3d final;

    /** @brief Get the current transform parameters */
    [[nodiscard]] auto params() const -> Parameters;
    /** @brief Set the transform parameters */
    void params(const Parameters& params);

    /**
     * @brief Add translation
     *
     * Per-axis translation function.
     */
    void translate(double x, double y, double z);

    /**
     * @brief Add translation
     *
     * Convenience function for `cv::Vec` types.
     */
    template <typename Tp, int Cn>
    void translate(const cv::Vec<Tp, Cn>& translation)
    {
        static_assert(Cn >= 3, "cv::Vec must have >= 3 values");
        translate(translation[0], translation[1], translation[2]);
    }

    /**
     * @brief Add rotation in degrees
     *
     * Rotate `angle` degrees around the vector `[ax, ay, az]`.
     */
    void rotate(double angle, double ax, double ay, double az);

    /**
     * @brief Add rotation in degrees
     *
     * @copydetails rotate(double,double,double,double)
     *
     * Convenience function for `cv::Vec` types.
     */
    template <typename Tp, int Cn>
    void rotate(double angle, cv::Vec<Tp, Cn> axis)
    {
        static_assert(Cn >= 3, "cv::Vec must have >= 3 values");
        rotate(angle, axis[0], axis[1], axis[2]);
    }

    /** @brief Add anisotropic scale */
    void scale(double sx, double sy, double sz);

    /** @brief Add isotropic scale */
    void scale(double s);

private:
    /** Don't allow construction on the stack */
    AffineTransform() = default;
    /** Current parameters */
    Parameters params_{cv::Matx<double, 4, 4>::eye()};
    /** @copydoc Transform3D::to_meta_() */
    void to_meta_(Metadata& meta) final;
    /** @copydoc Transform3D::from_meta_() */
    void from_meta_(const Metadata& meta) final;
};

/** @brief Apply a transform to an ITKMesh */
auto ApplyTransform(
    const ITKMesh::Pointer& mesh,
    const Transform3D::Pointer& transform,
    bool normalize = true) -> ITKMesh::Pointer;

/** @brief Apply a transform to a PerPixelMap */
auto ApplyTransform(
    const PerPixelMap& ppm,
    const Transform3D::Pointer& transform,
    bool normalize = true) -> PerPixelMap;

/** @brief Apply a transform to a PerPixelMap::Pointer */
auto ApplyTransform(
    const PerPixelMap::Pointer& ppm,
    const Transform3D::Pointer& transform,
    bool normalize = true) -> PerPixelMap::Pointer;

/** @brief Apply a transform to a PointSet */
template <class PointSetT>
auto ApplyTransform(const PointSetT& ps, const Transform3D::Pointer& transform)
    -> PointSetT;

}  // namespace volcart

#include "vc/core/types/TransformsImpl.hpp"