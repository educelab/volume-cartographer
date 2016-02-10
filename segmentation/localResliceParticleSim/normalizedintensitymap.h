#pragma once

#include <utility>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "common.h"

namespace volcart
{
namespace segmentation
{
// A class representing the intensity map generated from a row of a matrix
// normalized to the range [0, 1]
class NormalizedIntensityMap
{
public:
    NormalizedIntensityMap(cv::Mat);

    cv::Mat draw(const int32_t displayWidth = 100,
                 const int32_t displayHeight = 100) const;

    IndexIntensityPairVec sortedMaxima() const;

private:
    friend std::ostream& operator<<(std::ostream& s,
                                    const NormalizedIntensityMap& m)
    {
        return s << m.intensities_;
    }

    cv::Mat_<double> intensities_;
    double currentIntensity_;
    int32_t width_;
};
}
}
