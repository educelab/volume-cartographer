#pragma once

#include <deque>
#include <iostream>
#include <utility>

#include <opencv2/core.hpp>

#include "vc/segmentation/lrps/Common.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class IntensityMap
 * @brief A class representing the intensity map generated from a row of a
 * matrix, normalized to the range [0, 1]
 *
 * Attempts to classify and sort maxima relative to the horizontal center of
 * the matrix.
 * @ingroup lrps
 */
class IntensityMap
{
public:
    /** @name Constructors */
    /**@{*/
    /**
     * @param stepSize Row offset (from center) for generating intensity values
     * @param peakDistanceWeight Distance weight factor for maxima sorting
     * @param shouldIncludeMiddle Include center of selected row as maxima
     */
    IntensityMap(
        cv::Mat r,
        int stepSize,
        int peakDistanceWeight,
        bool shouldIncludeMiddle);
    /**@}*/

    /**
     * @brief Generate a rastered intensity map for debug
     */
    cv::Mat draw();

    /**
     * @brief Return the row's maxima, sorted by intensity and weighted by
     * distance from the center of the row
     */
    std::deque<std::pair<int, double>> sortedMaxima();

    /** @brief Select a maxima from the map by index */
    void setChosenMaximaIndex(int index) { chosenMaximaIndex_ = index; }

    /** @brief Return the currently selected maxima index */
    int chosenMaximaIndex() const { return chosenMaximaIndex_; }

    /** @brief Increase the maxima index by 1 */
    void incrementMaximaIndex() { chosenMaximaIndex_++; }

    /** @brief Return the peak radius */
    int peakRadius() const { return peakRadius_; }

private:
    friend std::ostream& operator<<(std::ostream& s, const IntensityMap& m)
    {
        return s << m.intensities_;
    }

    /** How much you want to move each time you leave an element */
    int stepSize_;

    /** Distance weight factor for maxima sorting */
    int peakDistanceWeight_;

    /** Unsorted intensity plot of selected row */
    cv::Mat_<double> intensities_;

    /** Input matrix */
    cv::Mat_<uint8_t> resliceData_;

    /** Width of the image returned by draw() */
    int displayWidth_;

    /** Height of the image returned by draw() */
    int displayHeight_;

    /** Image returned by draw() */
    cv::Mat drawTarget_;

    /** Width of the bin to hold the image */
    int binWidth_;

    /** Width of the intensity map */
    int mapWidth_;

    /** Maxima Index */
    int chosenMaximaIndex_;

    /** Include center of selected row as maxima? */
    bool shouldIncludeMiddle_;

    /** Largest distance a maxima can be away from center */
    const int peakRadius_ = 5;
};
}
}
