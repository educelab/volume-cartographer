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
    explicit SliceImage(boost::filesystem::path p) : path{std::move(p)} {}

    bool operator==(const SliceImage& b) const;
    bool operator!=(const SliceImage& b) const { return !operator==(b); }
    bool operator<(const SliceImage& b) const;

    bool analyze();
    cv::Mat conformedImage();
    int width() { return w_; }
    int height() { return h_; }
    int depth() { return depth_; }
    double min() { return min_; }
    double max() { return max_; }
    bool needsConvert() { return needsConvert_; }
    bool needsScale() { return needsScale_; }
    void setScale(double max, double min)
    {
        max_ = max;
        min_ = min;
    }

    boost::filesystem::path path;

private:
    int w_{0};
    int h_{0};
    int depth_{0};
    double min_{0};
    double max_{0};
    bool needsConvert_{false};
    bool needsScale_{false};
};
}