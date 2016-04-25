#pragma once

#include <utility>
#include <iostream>
#include <deque>
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
    IntensityMap(cv::Mat,
                 int32_t stepSize,
                 int32_t peakDistanceWeight,
                 bool shouldIncludeMiddle);

    cv::Mat draw();

    std::deque<std::pair<int32_t, double>> sortedMaxima();

    void setChosenMaximaIndex(int32_t index) { chosenMaximaIndex_ = index; }

    int32_t chosenMaximaIndex() const { return chosenMaximaIndex_; }

    void incrementMaximaIndex() { chosenMaximaIndex_++; }

    int32_t peakRadius() const { return peakRadius_; }

private:
    friend std::ostream& operator<<(std::ostream& s, const IntensityMap& m)
    {
        return s << m.intensities_;
    }

    int32_t stepSize_;
    int32_t peakDistanceWeight_;
    cv::Mat_<double> intensities_;
    cv::Mat_<uint8_t> resliceData_;
    int32_t displayWidth_;
    int32_t displayHeight_;
    cv::Mat drawTarget_;
    int32_t binWidth_;
    int32_t mapWidth_;
    int32_t chosenMaximaIndex_;
    bool shouldIncludeMiddle_;

    const int32_t peakRadius_ = 5;
};
}
}
