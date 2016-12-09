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

#include "core/types/UVMap.h"

namespace volcart
{
class PerPixelMap
{
public:
    ///// Constructors /////
    // Create empty
    PerPixelMap() : _width{0}, _height{0} {}

    // Create new
    PerPixelMap(size_t height, size_t width);

    ///// Check if initialized /////
    bool initialized() const { return _map.data && _width > 0 && _height > 0; };

    ///// Operators /////
    // Forward to the Mat_ operators
    cv::Vec6d& operator()(size_t y, size_t x) { return _map(y, x); };

    ///// Metadata /////
    void setDimensions(size_t h, size_t w);
    void setWidth(size_t w);
    void setHeight(size_t h);
    int width() const { return _width; };
    int height() const { return _height; };

    void setUVMap(UVMap u) { _uvmap = u; };
    const UVMap& uvMap() const { return _uvmap; };
    UVMap& uvMap() { return _uvmap; };

    void setMask(cv::Mat m) { _mask = m.clone(); };
    cv::Mat mask() const { return _mask; };
    cv::Mat maskCopy() const { return _mask.clone(); };
    bool hasMapping(size_t y, size_t x);

    ///// Disk IO /////
    static void WritePPM(boost::filesystem::path path, const PerPixelMap& map);
    static PerPixelMap ReadPPM(boost::filesystem::path path);

private:
    void _initializeMap();
    size_t _width, _height;
    cv::Mat_<cv::Vec6d> _map;
    cv::Mat _mask;
    UVMap _uvmap;
};
}
