#pragma once

/** @file */

#include <cstddef>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

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
    /** @brief Transform type string constant */
    static constexpr std::string_view TYPE{"Transform3D"};

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
    [[nodiscard]] virtual auto type() const -> std::string_view = 0;
    /** @brief Clone the transform */
    [[nodiscard]] virtual auto clone() const -> Pointer = 0;

    /** @brief Return whether the underlying transform is invertible */
    [[nodiscard]] virtual auto invertible() const -> bool;
    /** @brief Return the inverted transform */
    [[nodiscard]] virtual auto invert() const -> Pointer;
    /** @brief Return whether the underlying transform is composable */
    [[nodiscard]] virtual auto composable() const -> bool;

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
    void clear();

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

    /**
     * @brief Compose two transforms into a single new transform
     *
     * Returns a pair of transform pointers. If the composition fails, the pair
     * will contain pointers to both of the original inputs. If the second
     * value in the pair is nullptr, then the composition was successful, and
     * the new transform is available from the first pointer.
     *
     * @code{.cpp}
     * // Two transforms
     * Transform3D::Pointer lhs = AffineTransform::New();
     * Transform3D::Pointer rhs = AffineTransform::New();
     *
     * // Compose and assign the results to existing variables
     * std::tie(lhs, rhs) = Transform3D::Compose(lhs, rhs);
     *
     * // Check the result
     * if(rhs) {
     *   std::cout << "Failed to compose transforms!\n";
     * } else {
     *   std::cout << "Composition successful!\n";
     * }
     * @endcode
     */
    static auto Compose(const Pointer& lhs, const Pointer& rhs)
        -> std::pair<Pointer, Pointer>;

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

    /**
     * Helper compose function. The base implementation returns a nullptr.
     * Implementing derived classes should set the returned transform's source
     * to this->source() and the target to rhs->target().
     */
    [[nodiscard]] virtual auto compose_(const Pointer& rhs) const -> Pointer;

    /** On-disk metadata type */
    using Metadata = nlohmann::ordered_json;
    /** Serialize the transform to metadata */
    static auto Serialize(const Pointer& transform) -> Metadata;
    /** Deserialize the transform from metadata */
    static auto Deserialize(const Metadata& meta) -> Pointer;
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
 * @brief Compose transform convenience operator
 *
 * Same as Transform3D::Compose but only returns the composed transform. If
 * composition fails for any reason, will throw an exception.
 *
 * @throws std::invalid_argument if lhs or rhs are not composable
 * @throws std::runtime_error if transform composition failed
 */
auto operator*(const Transform3D::Pointer& lhs, const Transform3D::Pointer& rhs)
    -> Transform3D::Pointer;

/**
 * @brief 3D affine transform
 *
 * Initializes to an identity. Modify the transform by calling the translate(),
 * rotate(), and scale() functions. Each call precomposes the function's
 * transform with the stored affine transform. For example, the following
 * transform will scale, rotate, and translate the 3D point, in that order:
 *
 * @code{.cpp}
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
    /** @copydoc Transform3D::TYPE */
    static constexpr std::string_view TYPE{"AffineTransform"};

    /** Parameters type: 4x4 matrix */
    using Parameters = cv::Matx<double, 4, 4>;

    /** Pointer type */
    using Pointer = std::shared_ptr<AffineTransform>;

    /** @brief Create a new AffineTransform */
    static auto New() -> Pointer;

    /** @copydoc Transform3D::type() */
    [[nodiscard]] auto type() const -> std::string_view final;
    /** @copydoc Transform3D::clone() */
    [[nodiscard]] auto clone() const -> Transform3D::Pointer final;
    /** @copydoc Transform3D::invertible() */
    [[nodiscard]] auto invertible() const -> bool final;
    /** @copydoc Transform3D::invert() */
    [[nodiscard]] auto invert() const -> Transform3D::Pointer final;
    /** @copydoc Transform3D::composable() */
    [[nodiscard]] auto composable() const -> bool final;
    /** @copydoc Transform3D::reset() */
    void reset() final;

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
    void rotate(const double angle, cv::Vec<Tp, Cn> axis)
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
    Parameters params_{Parameters::eye()};
    /** @copydoc Transform3D::compose_() */
    [[nodiscard]] auto compose_(const Transform3D::Pointer& rhs) const
        -> Transform3D::Pointer final;
    /** @copydoc Transform3D::to_meta_() */
    void to_meta_(Metadata& meta) final;
    /** @copydoc Transform3D::from_meta_() */
    void from_meta_(const Metadata& meta) final;
};

/**
 * @brief Identity transform
 *
 * Identity transform that simply returns input parameters. Useful for
 * creating explicit mappings between a source and a target which share the
 * same coordinate space.
 *
 * @code{.cpp}
 * auto tfm = IdentityTransform::New();
 * auto pt = tfm->applyPoint({0, 1, 0}); // {0, 1, 0}
 * @endcode
 */
class IdentityTransform : public Transform3D
{
public:
    /** @copydoc Transform3D::TYPE */
    static constexpr std::string_view TYPE{"IdentityTransform"};

    /** Pointer type */
    using Pointer = std::shared_ptr<IdentityTransform>;

    /** @brief Create a new IdentityTransform */
    static auto New() -> Pointer;

    /** @copydoc Transform3D::type() */
    [[nodiscard]] auto type() const -> std::string_view final;
    /** @copydoc Transform3D::clone() */
    [[nodiscard]] auto clone() const -> Transform3D::Pointer final;
    /** @copydoc Transform3D::invertible() */
    [[nodiscard]] auto invertible() const -> bool final;
    /** @copydoc Transform3D::invert() */
    [[nodiscard]] auto invert() const -> Transform3D::Pointer final;
    /** @copydoc Transform3D::composable() */
    [[nodiscard]] auto composable() const -> bool final;
    /** @copydoc Transform3D::reset() */
    void reset() final;

    /** @copydoc Transform3D::applyPoint() */
    [[nodiscard]] auto applyPoint(const cv::Vec3d& point) const
        -> cv::Vec3d final;
    /** @copydoc Transform3D::applyVector() */
    [[nodiscard]] auto applyVector(const cv::Vec3d& vector) const
        -> cv::Vec3d final;

private:
    /** Don't allow construction on the stack */
    IdentityTransform() = default;
    /** @copydoc Transform3D::compose_() */
    [[nodiscard]] auto compose_(const Transform3D::Pointer& rhs) const
        -> Transform3D::Pointer final;
    /** @copydoc Transform3D::to_meta_() */
    void to_meta_(Metadata& meta) final;
    /** @copydoc Transform3D::from_meta_() */
    void from_meta_(const Metadata& meta) final;
};

/**
 * @brief Collection of transforms
 *
 * A convenience class which holds a list of transforms. When transforming
 * points and vectors, each transform is applied sequentially to the input.
 *
 * @code{.cpp}
 * // New transform
 * auto tfm = CompositeTransform::New();
 *
 * // Add some transforms
 * auto t = AffineTransform::New();
 * t->translate(1, 2, 3);
 * tfm->push_back(t);
 * t->reset();
 * t->scale(4);
 * tfm->push_back(t);
 *
 * // Apply all transforms to an input
 * auto pt = tfm->applyPoint({0, 1, 0}); // {4, 12, 12}
 * @endcode
 *
 * It can often be preferable, for both performance and numerical stability, to
 * simplify all adjacent, composable transforms (e.g. AffineTransform,
 * IdentityTransform) into a single transform.
 *
 * @code{.cpp}
 * // Add some composable transforms
 * tfm->push_back(AffineTransform::New());
 * tfm->push_back(IdentityTransform::New());
 * tfm->push_back(AffineTransform::New());
 * tfm->size(); // 3
 *
 * // Simplify the transform
 * tfm->simplify();
 * tfm->size(); // 1
 * @endcode
 */
class CompositeTransform : public Transform3D
{
public:
    /** @copydoc Transform3D::TYPE */
    static constexpr std::string_view TYPE{"CompositeTransform"};

    /** Pointer type */
    using Pointer = std::shared_ptr<CompositeTransform>;

    /** @brief Create a new CompositeTransform */
    static auto New() -> Pointer;

    /** @copydoc Transform3D::type() */
    [[nodiscard]] auto type() const -> std::string_view final;
    /** @copydoc Transform3D::clone() */
    [[nodiscard]] auto clone() const -> Transform3D::Pointer final;
    /** @copydoc Transform3D::reset() */
    void reset() final;

    /** @copydoc Transform3D::applyPoint() */
    [[nodiscard]] auto applyPoint(const cv::Vec3d& point) const
        -> cv::Vec3d final;
    /** @copydoc Transform3D::applyVector() */
    [[nodiscard]] auto applyVector(const cv::Vec3d& vector) const
        -> cv::Vec3d final;

    /**
     * @brief Add a transform to the front of the composite transform stack
     *
     * The transform is cloned before being added to the transform stack. If
     * the transform is also a CompositeTransform, its transform stack is
     * expanded and copied to the front of this transform's stack.
     */
    void push_front(const Transform3D::Pointer& t);

    /**
     * @brief Add a transform to the end of the composite transform stack
     *
     * The transform is cloned before being added to the transform stack. If
     * the transform is also a CompositeTransform, its transform stack is
     * expanded and copied to the end of this transform's stack.
     */
    void push_back(const Transform3D::Pointer& t);

    /** @brief Get the number of transforms in the composite transform */
    [[nodiscard]] auto size() const noexcept -> std::size_t;

    /**
     * @brief Compose all composable transforms
     *
     * Simplifies the transform by composing all adjacent, composable
     * transforms in the composite transform list. This can lead to better
     * runtime performance and numerical stability for the apply functions.
     */
    void simplify();

    /** @brief Get a list of the stored transforms */
    [[nodiscard]] auto transforms() const -> std::vector<Transform3D::Pointer>;

private:
    /** Don't allow construction on the stack */
    CompositeTransform() = default;
    /** Transform list */
    std::list<Transform3D::Pointer> tfms_;
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