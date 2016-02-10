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
    IntensityMap(cv::Mat);

    cv::Mat draw();
    IndexIntensityPairVec sortedMaxima() const;

    void setFinalChosenMaximaIndex(const int32_t index)
    {
        chosenMaximaIndex_ = index;
    }

    void incrementFinalChosenMaximaIndex(void) { chosenMaximaIndex_++; }
private:
    friend std::ostream& operator<<(std::ostream& s, const IntensityMap& m)
    {
        return s << m.intensities_;
    }

    cv::Mat_<double> intensities_;
    int32_t displayWidth_;
    int32_t displayHeight_;
    cv::Mat drawTarget_;
    int32_t binWidth_;
    double currentIntensity_;
    int32_t mapWidth_;
    int32_t chosenMaximaIndex_;
};
}
}
