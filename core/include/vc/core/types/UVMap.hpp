// VC UV Map
// Object to store UV mappings for points in a mesh.
// Created by Seth Parker on 10/20/15.

// Internally, all mappings are stored relative to the "top-left" of the 2D
// mapping space. get() & set() transform
// values to and from this relative range based on the value of origin_. By
// setting origin_ prior to insertion and
// again prior to retrieval, mappings can be inserted relative to one origin but
// retrieved relative to another.
#pragma once

#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/vc_defines.hpp"

constexpr static int VC_UVMAP_MIN_DEBUG_WIDTH = 500;

namespace volcart
{

class UVMap
{
public:
    UVMap() : origin_{VC_ORIGIN_TOP_LEFT} {}
    explicit UVMap(Origin o) : origin_{o} {}

    size_t size() const { return map_.size(); }
    bool empty() const { return map_.empty(); }

    // Get and set origin
    Origin origin() const { return origin_; }
    void origin(const Origin& o) { origin_ = o; }

    // Set the uv mapping for point id
    void set(size_t id, const cv::Vec2d& uv)
    {
        // transform to be relative to top-left
        cv::Vec2d transformed;
        cv::absdiff(uv, origin_, transformed);
        map_[id] = transformed;
    }

    // Get the uv mapping for point id
    cv::Vec2d get(size_t id)
    {
        auto it = map_.find(id);
        if (it != map_.end()) {
            // transform to be relative to origin_
            cv::Vec2d transformed;
            cv::absdiff(it->second, origin_, transformed);
            return transformed;
        } else {
            return VC_UVMAP_NULL_MAPPING;
        }
    }

    // Ratio information
    Ratio ratio() const { return ratio_; }
    void ratio(double a) { ratio_.aspect = a; }
    void ratio(double w, double h)
    {
        ratio_.width = w;
        ratio_.height = h;
        ratio_.aspect = w / h;
    }

    // Plot the UV points on a cv::Mat img
    cv::Mat drawUVMap() const
    {
        auto w = static_cast<int>(std::ceil(ratio_.width));
        if (w < VC_UVMAP_MIN_DEBUG_WIDTH) {
            std::cerr
                << "volcart::UVMap:: Width less than minimum. Scaling image."
                << std::endl;
            w = VC_UVMAP_MIN_DEBUG_WIDTH;
        }
        auto h = static_cast<int>(std::ceil(w / ratio_.aspect));
        cv::Mat r = cv::Mat::zeros(h, w, CV_8UC1);

        for (auto it : map_) {
            cv::Point2d p(it.second[0] * w, it.second[1] * h);
            cv::circle(r, p, 2, 255, -1);
        }

        return r;
    }

private:
    std::unordered_map<size_t, cv::Vec2d> map_;
    // origin inserted and retrieved points are relative to
    cv::Vec2d origin_;
    Ratio ratio_;
};
}
