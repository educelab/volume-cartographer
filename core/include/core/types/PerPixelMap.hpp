///// Per-Pixel Map /////
//
// Effectively a raster of a UV Map for reverse lookups.
// Every pixel in the map holds the 3D position and normal
// used to generate that pixel's intensity in the corresponding
// texture image. Useful for regenerating textures with differing
// parameters or for identifying where in a volume a particular
// came from.
//
// Created by Seth Parker on 3/17/16.
#pragma once

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "core/io/PointSetIO.hpp"
#include "core/types/OrderedPointSet.hpp"
#include "core/types/UVMap.hpp"

namespace volcart
{
class PerPixelMap
{
public:
    ///// Constructors /////
    // Create empty
    PerPixelMap() : width_{0}, height_{0} { initialize_map_(); }

    // Create new
    PerPixelMap(size_t height, size_t width) : width_{width}, height_{height}
    {
        initialize_map_();
    }

    ///// Check if initialized /////
    bool initialized() const
    {
        return width_ == map_.width() && height_ == map_.height() &&
               width_ > 0 && height_ > 0;
    }

    ///// Operators /////
    // Forward to the Mat_ operators
    cv::Vec6d& operator()(size_t y, size_t x) { return map_(x, y); }

    ///// Metadata /////
    void setDimensions(size_t h, size_t w);
    void setWidth(size_t w);
    void setHeight(size_t h);
    int width() const { return width_; }
    int height() const { return height_; }

    void setUVMap(const UVMap& u) { uvMap_ = u; }
    const UVMap& uvMap() const { return uvMap_; }
    UVMap& uvMap() { return uvMap_; }

    void setMask(const cv::Mat& m) { mask_ = m.clone(); }
    cv::Mat mask() const { return mask_; }
    cv::Mat maskCopy() const { return mask_.clone(); }
    bool hasMapping(size_t y, size_t x)
    {
        return mask_.at<uint8_t>(y, x) == 255;
    }

    ///// Disk IO /////
    static void WritePPM(boost::filesystem::path path, const PerPixelMap& map);
    static PerPixelMap ReadPPM(const boost::filesystem::path& path);

private:
    void initialize_map_();
    size_t width_, height_;
    volcart::OrderedPointSet<cv::Vec6d> map_;
    cv::Mat mask_;
    UVMap uvMap_;
};
}  // namespace volcart
