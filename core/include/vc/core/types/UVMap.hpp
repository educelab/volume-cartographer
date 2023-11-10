#pragma once

/** @file */

#include <map>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/types/Color.hpp"
#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
/** Null/Undefined UV Mapping */
const static cv::Vec2d NULL_MAPPING{-1, -1};

/**
 * @class UVMap
 * @author Seth Parker
 * @date 10/20/15
 *
 * @brief Stores per-vertex UV mappings
 *
 * A UV map stores the position of a vertex in 2D parameter space (often texture
 * space). UV values are typically stored as floating point values in the range
 * `[0,1]`.
 *
 * UV positions are assumed to be relative to an origin at one of the corners of
 * parameter space: the top-left `(0,0)`, top-right `(1,0)`, bottom-left
 * `(0,1)`, or bottom-right `(1,1)`. Internally, all mappings are stored
 * relative to a storage origin. The set() functions transform UV values from
 * the provided origin space to the storage origin space. Similarly, the get()
 * functions transform UV values from the storage origin space to the provided
 * origin space. This allows the class to provide on-the-fly conversion between
 * different origin positions. By setting the origin prior to insertion and
 * again prior to retrieval, mappings can be inserted relative to one origin but
 * retrieved relative to another. When using the overloaded
 * set(size_t, const cv::Vec2d&) and get(size_t) functions, the source and
 * target origins are set using the constructor or setOrigin().
 *
 * Since UV maps store \em relative position information, they are agnostic to
 * size of the texture space to which they apply. The ratio functions provide
 * a way to store the dimensions and aspect ratio of the texture space for
 * later PerPixelMap and Texture generation.
 *
 * @ingroup Types
 */
class UVMap
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<UVMap>;

    /** Origin corner position enumeration */
    enum class Origin { TopLeft = 0, TopRight, BottomLeft, BottomRight };

    /** Aspect ratio structure */
    struct Ratio {
        double width{1}, height{1}, aspect{1};
    };

    /**@{*/
    /** @brief Default constructor */
    UVMap() = default;

    /** @brief Construct and set origin */
    explicit UVMap(Origin o);

    /** Static New function for all constructors of T */
    template <typename... Args>
    static auto New(Args... args) -> Pointer
    {
        return std::make_shared<UVMap>(std::forward<Args>(args)...);
    }
    /**@}*/

    /**@{*/
    /** @brief Return the number of UV elements */
    [[nodiscard]] auto size() const -> size_t;

    /** @brief Return whether the UVMap is empty */
    [[nodiscard]] auto empty() const -> bool;
    /**@}*/

    /**@{*/
    /** @brief Set the origin of the UVMap
     *
     * UV values inserted and retrieved after a call to setOrigin() will be
     * relative to this value.
     */
    void setOrigin(const Origin& o);

    /** @brief Get the current origin of the UVMap */
    [[nodiscard]] auto origin() const -> Origin;
    /**@}*/

    /**@{*/
    /**
     * @brief Set the UV value for a point by ID
     *
     * Point is inserted relative to the provided origin.
     */
    void set(size_t id, const cv::Vec2d& uv, const Origin& o);

    /**
     * @copybrief set()
     *
     * Point is inserted relative to the origin returned by origin().
     */
    void set(size_t id, const cv::Vec2d& uv);

    /**
     * @brief Get the UV value for a point by ID
     *
     * Point is retrieved relative to the provided origin.
     */
    [[nodiscard]] auto get(size_t id, const Origin& o) const -> cv::Vec2d;

    /**
     * @copybrief get()
     *
     * Point is retrieved relative to the origin returned by origin().
     */
    [[nodiscard]] auto get(size_t id) const -> cv::Vec2d;

    /** @brief Check if the vertex index has a UV mapping */
    [[nodiscard]] auto contains(std::size_t id) const -> bool;

    /** @brief Clear map. */
    void clear_map();

    /** Access to underlying data. For serialization only. */
    [[nodiscard]] auto as_map() const -> std::map<size_t, cv::Vec2d>;
    /**@}*/

    /**@{*/
    /** @brief Get the size information (aspect ratio, width, height) */
    [[nodiscard]] auto ratio() const -> Ratio;

    /** @brief Set the aspect ratio */
    void ratio(double a);

    /** @brief Set the aspect ration by width and height parameters */
    void ratio(double w, double h);
    /**@}*/

    /**@{*/
    /** Rotation preset enumeration */
    enum class Rotation { CW90 = 0, CW180, CCW90 };

    /** Flipping axis enumeration */
    enum class FlipAxis { Vertical = 0, Horizontal, Both };

    /** Align to axis enumeration */
    enum class AlignmentAxis { None = 0, ZPos, ZNeg, YPos, YNeg, XPos, XNeg };

    /**
     * @brief Plot the UV points on an image
     *
     * For debug purposes only.
     */
    static auto Plot(const UVMap& uv, const Color& color = color::GREEN)
        -> cv::Mat;

    /**
     * @brief Plot the UV mesh on an image
     *
     * For debug purposes only.
     */
    static auto Plot(
        const UVMap& uv,
        const ITKMesh::Pointer& mesh2D,
        int width = -1,
        int height = -1,
        const Color& color = color::LIGHT_GRAY) -> cv::Mat;

    /** @brief Align a UVMap to a specified volume axis */
    static void AlignToAxis(
        UVMap& uv, const ITKMesh::Pointer& mesh, AlignmentAxis axis);

    /** @brief Rotate a UVMap by a multiple of 90 degrees */
    static void Rotate(UVMap& uv, Rotation rotation);

    /**
     * @copydoc Rotate(UVMap&, Rotation)
     *
     * This function is an overload which also rotates the texture image.
     */
    static void Rotate(UVMap& uv, Rotation rotation, cv::Mat& texture);

    /**
     * @brief Rotate a UVMap by a specified angle
     *
     * Theta is in radians. Rotation is performed in UV space and is
     * counter-clockwise relative to the provided `center` position.
     */
    static void Rotate(
        UVMap& uv, double theta, const cv::Vec2d& center = {0.5, 0.5});

    /**
     * @copydoc Rotate(UVMap& uv, double, const cv::Vec2d&)
     *
     * This function is an overload which also rotates the texture image.
     */
    static void Rotate(
        UVMap& uv,
        double theta,
        cv::Mat& texture,
        const cv::Vec2d& center = {0.5, 0.5});

    /** @brief Flip a UVMap across one or both of its axes */
    static void Flip(UVMap& uv, FlipAxis axis);
    /**@}*/

private:
    /** UV storage */
    std::map<size_t, cv::Vec2d> map_;
    /** Origin for set and get functions */
    Origin origin_{Origin::TopLeft};
    /** Aspect ratio */
    Ratio ratio_;
};
}  // namespace volcart
