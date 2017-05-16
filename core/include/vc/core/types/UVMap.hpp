#pragma once

#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/vc_defines.hpp"

namespace volcart
{
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
    /**@{*/
    /** @brief Construct and set origin */
    explicit UVMap(const Origin& o = VC_ORIGIN_TOP_LEFT) : origin_{o} {}
    /**@}*/

    /**@{*/
    /** @brief Return the number of UV elements */
    size_t size() const { return map_.size(); }

    /** @brief Return whether the UVMap is empty */
    bool empty() const { return map_.empty(); }
    /**@}*/

    /**@{*/
    /** @brief Set the origin of the UVMap
     *
     * UV values inserted and retrieved after a call to setOrigin() will be
     * relative to his value.
     */
    void setOrigin(const Origin& o) { origin_ = o; }

    /** @brief Get the current origin of the UVMap */
    Origin origin() const { return origin_; }
    /**@}*/

    /**@{*/
    /**
     * @brief Set the UV value for a point by ID
     *
     * Point is inserted relative to the provided origin.
     */
    void set(size_t id, const cv::Vec2d& uv, const Origin& o)
    {
        // transform to be relative to top-left
        cv::Vec2d transformed;
        cv::absdiff(uv, o, transformed);
        map_[id] = transformed;
    }

    /**
     * @copybrief set()
     *
     * Point is inserted relative to the origin returned by origin().
     */
    void set(size_t id, const cv::Vec2d& uv) { set(id, uv, origin_); }

    /**
     * @brief Get the UV value for a point by ID
     *
     * Point is retrieved relative to the provided origin.
     */
    cv::Vec2d get(size_t id, const Origin& o)
    {
        auto it = map_.find(id);
        if (it != map_.end()) {
            // transform to be relative to the provided origin
            cv::Vec2d transformed;
            cv::absdiff(it->second, o, transformed);
            return transformed;
        } else {
            return VC_UVMAP_NULL_MAPPING;
        }
    }

    /**
     * @copybrief get()
     *
     * Point is retrieved relative to the origin returned by origin().
     */
    cv::Vec2d get(size_t id) { return get(id, origin_); }
    /**@}*/

    /**@{*/
    /** @brief Get the size information (aspect ratio, width, height) */
    Ratio ratio() const { return ratio_; }

    /** @brief Set the aspect ratio */
    void ratio(double a) { ratio_.aspect = a; }

    /** @brief Set the aspect ration by width and height parameters */
    void ratio(double w, double h)
    {
        ratio_.width = w;
        ratio_.height = h;
        ratio_.aspect = w / h;
    }
    /**@}*/

    /**@{*/
    /** Minimum size of a UVMap debug image */
    constexpr static int MIN_DEBUG_WIDTH = 500;

    /**
     * @brief Plot the UV points on an image
     *
     * For debug purposes only.
     */
    cv::Mat drawUVMap() const
    {
        auto w = static_cast<int>(std::ceil(ratio_.width));
        if (w < MIN_DEBUG_WIDTH) {
            w = MIN_DEBUG_WIDTH;
        }
        auto h = static_cast<int>(std::ceil(w / ratio_.aspect));
        cv::Mat r = cv::Mat::zeros(h, w, CV_8UC1);

        for (auto it : map_) {
            cv::Point2d p(it.second[0] * w, it.second[1] * h);
            cv::circle(r, p, 2, 255, -1);
        }

        return r;
    }
    /**@}*/

private:
    /** UV storage */
    std::unordered_map<size_t, cv::Vec2d> map_;
    /** Origin for set and get functions */
    cv::Vec2d origin_;
    /** Aspect ratio */
    Ratio ratio_;
};
}
