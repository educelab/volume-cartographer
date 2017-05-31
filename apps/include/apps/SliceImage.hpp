#pragma once

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

namespace volcart
{

class SliceImage
{
public:
    SliceImage(boost::filesystem::path p) : path{std::move(p)} {}

    bool operator==(const SliceImage& b) const;
    bool operator!=(const SliceImage& b) const { return !operator==(b); }
    bool operator<(const SliceImage& b) const;

    bool analyze();
    cv::Mat conformedImage();
    int width() { return _w; }
    int height() { return _h; }
    int depth() { return _depth; }
    double min() { return _min; }
    double max() { return _max; }
    bool needsConvert() { return _convert; }

    boost::filesystem::path path;

private:
    int _w, _h, _depth;
    double _min, _max;
    bool _convert = false;  // true if file needs to be converted
};
}