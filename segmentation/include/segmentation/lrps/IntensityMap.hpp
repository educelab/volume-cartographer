#pragma once

#include <deque>
#include <iostream>
#include <utility>

#include <opencv2/core.hpp>

#include "segmentation/lrps/Common.hpp"

namespace volcart
{
namespace segmentation
{
/**
 * @class IntensityMap
 * @brief A class representing the intensity map generated from a row of a
 * matrix, normalized to the range [0, 1]
 * @ingroup lrps
 */
class IntensityMap
{
public:
    //** @name Constructors */
    //@{
    /**
     * @brief
     * @param stepSize Row offset (from center) for generating intensity values
     * @param peakDistanceWeight How much the distance between points
     *                           should be taken into account
     * @param shouldIncludeMiddle Whether or not to include points
     *                            in the middle
     */
    IntensityMap(
        cv::Mat r,
        int stepSize,
        int peakDistanceWeight,
        bool shouldIncludeMiddle);
    //@}

    /**
     * @brief Creates the intensity map
     */
    cv::Mat draw();

    /**
     * @brief The top maxima in the row being processed, number of maxima
     * determined by the peak radius
     */
    std::deque<std::pair<int, double>> sortedMaxima();

    /**
     * @brief Sets the Maxima Index
     * @param index What you want the Maxima Index to be
     */
    void setChosenMaximaIndex(int index) { chosenMaximaIndex_ = index; }

    /** @brief Returns the current Maxima Index*/
    int chosenMaximaIndex() const { return chosenMaximaIndex_; }

    /** @brief Increases the Maxima Index by 1 */
    void incrementMaximaIndex() { chosenMaximaIndex_++; }

    /** @brief Returns the Peak Radius  */
    int peakRadius() const { return peakRadius_; }

private:
    /**@brief Overwrites output operator to write the intensities  */
    friend std::ostream& operator<<(std::ostream& s, const IntensityMap& m)
    {
        return s << m.intensities_;
    }
    /** How much you want to move each time you leave an element */
    int stepSize_;
    /** How much the distance between two points should be taken
     * into account */
    int peakDistanceWeight_;
    /** List of Intensities in the Image*/
    cv::Mat_<double> intensities_;
    /** Image of the data from when the imaged was resliced */
    cv::Mat_<uint8_t> resliceData_;
    /** Width of the image when it's displayed*/
    int displayWidth_;
    /** Height of the image when it's displayed*/
    int displayHeight_;
    /** Where the Intensity map is saved*/
    cv::Mat drawTarget_;
    /** Width of the bin to hold the image*/
    int binWidth_;
    /** Width of the intensity map*/
    int mapWidth_;
    /** Maxima Index*/
    int chosenMaximaIndex_;
    /** Determines whether or not to include pixels in the middle*/
    bool shouldIncludeMiddle_;
    /** Largest radius when searching a neighborhood of pixel*/
    const int peakRadius_ = 5;
};
}
}
