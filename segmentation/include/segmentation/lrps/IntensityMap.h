#pragma once

#include <deque>
#include <iostream>
#include <utility>

#include <opencv2/core.hpp>

#include "segmentation/lrps/Common.h"

namespace volcart
{
namespace segmentation
{
// A class representing the intensity map generated from a row of a matrix
// normalized to the range [0, 1]
class IntensityMap
{
public:
    IntensityMap(
        cv::Mat r,
        int stepSize,
        int peakDistanceWeight,
        bool shouldIncludeMiddle);

    cv::Mat draw();

    std::deque<std::pair<int, double>> sortedMaxima();

    void setChosenMaximaIndex(int index) { chosenMaximaIndex_ = index; }

    int chosenMaximaIndex() const { return chosenMaximaIndex_; }

    void incrementMaximaIndex() { chosenMaximaIndex_++; }

    int peakRadius() const { return peakRadius_; }

private:
    friend std::ostream& operator<<(std::ostream& s, const IntensityMap& m)
    {
        return s << m.intensities_;
    }

    int stepSize_;
    int peakDistanceWeight_;
    cv::Mat_<double> intensities_;
    cv::Mat_<uint8_t> resliceData_;
    int displayWidth_;
    int displayHeight_;
    cv::Mat drawTarget_;
    int binWidth_;
    int mapWidth_;
    int chosenMaximaIndex_;
    bool shouldIncludeMiddle_;

    const int peakRadius_ = 5;
};
}
}
