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
class IntensityMap
{
public:
    IntensityMap(cv::Mat, const int32_t stepSize);

    cv::Mat draw();

    IndexIntensityPairVec sortedMaxima();

    void setChosenMaximaIndex(const int32_t index)
    {
        chosenMaximaIndex_ = index;
    }

    int32_t chosenMaximaIndex(void) const { return chosenMaximaIndex_; }

private:
    friend std::ostream& operator<<(std::ostream& s, const IntensityMap& m)
    {
        return s << m.intensities_;
    }

    int32_t stepSize_;
    cv::Mat_<double> intensities_;
    cv::Mat_<uint8_t> resliceData_;
    int32_t displayWidth_;
    int32_t displayHeight_;
    cv::Mat drawTarget_;
    int32_t binWidth_;
    int32_t mapWidth_;
    int32_t chosenMaximaIndex_;
};
}
}
