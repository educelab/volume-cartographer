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
#include <opencv2/opencv.hpp>
#include "common/types/UVMap.h"

namespace volcart
{
class PerPixelMap
{
public:
    ///// Constructors /////
    // Create empty
    PerPixelMap() : _width(0), _height(0){};

    // Create new
    PerPixelMap(int height, int width);

    // Construct map from file
    PerPixelMap(boost::filesystem::path path);

    ///// Check if initialized /////
    bool initialized() const { return _map.data && _width > 0 && _height > 0; };

    ///// Operators /////
    // Forward to the Mat_ operators
    cv::Vec6d& operator()(int y, int x) { return _map(y, x); };

    ///// Metadata /////
    void setDimensions(int w, int h);
    void setWidth(int w);
    void setHeight(int h);
    int width() const { return _width; };
    int height() const { return _height; };

    void setUVMap(UVMap u) { _uvmap = u; };
    const UVMap& getUVMap() const { return _uvmap; };
    UVMap& getUVMap() { return _uvmap; };

    void setMask(cv::Mat m) { _mask = m.clone(); };
    cv::Mat getMask() const { return _mask; };
    cv::Mat getMaskCopy() const { return _mask.clone(); };
    bool hasMapping(int y, int x);

    ///// Disk IO /////
    void write(boost::filesystem::path path);
    void read(boost::filesystem::path path);

private:
    void _initializeMap();
    int _width, _height;
    cv::Mat_<cv::Vec6d> _map;
    cv::Mat _mask;
    UVMap _uvmap;
};
}
