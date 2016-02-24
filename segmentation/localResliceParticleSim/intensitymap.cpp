#include "intensitymap.h"
#include "derivative.h"
#include <limits>

using namespace volcart::segmentation;

IntensityMap::IntensityMap(cv::Mat r)
    : displayWidth_(200),
      displayHeight_(200),
      drawTarget_(displayWidth_, displayHeight_, CV_8UC3, BGR_BLACK),
      chosenMaximaIndex_(-1)
{
    // DEBUG - need to convert to 8 bit before we can do anything
    r.convertTo(r, CV_8UC1, 1.0 / std::numeric_limits<uint8_t>::max());
    cv::equalizeHist(r, r);

    cv::normalize(r, intensities_, 0, 1, CV_MINMAX, CV_64FC1);
    mapWidth_ = intensities_.cols;
    binWidth_ = cvRound(float(displayWidth_) / mapWidth_);
    currentIntensity_ = intensities_(mapWidth_ / 2);
}

cv::Mat IntensityMap::draw()
{
    // Repaint the drawTarget_ so we don't draw over others
    drawTarget_ = BGR_BLACK;

    // Build intensity map
    for (int32_t i = 1; i < mapWidth_; ++i) {
        auto p1 = cv::Point(
            binWidth_ * (i - 1),
            displayHeight_ - cvRound(displayHeight_ * intensities_(i - 1)));
        auto p2 = cv::Point(
            binWidth_ * i,
            displayHeight_ - cvRound(displayHeight_ * intensities_(i)));

        // Draw a point at each intensity on the curve
        auto pointColor = (i == mapWidth_ / 2 ? BGR_YELLOW : BGR_RED);
        cv::circle(drawTarget_,
                   {binWidth_ * i,
                    displayHeight_ - cvRound(displayWidth_ * intensities_(i))},
                   2, pointColor);
        cv::line(drawTarget_, p1, p2, BGR_GREEN);
    }
    auto maxima = sortedMaxima();

    // A line for each candidate position
    for (const auto m : maxima) {
        cv::line(drawTarget_, cv::Point(binWidth_ * m.first, 0),
                 cv::Point(binWidth_ * m.first, drawTarget_.rows), BGR_BLUE);
    }

    // A Line for top maxima
    if (!maxima.empty()) {
        cv::line(drawTarget_, cv::Point(binWidth_ * maxima[0].first, 0),
                 cv::Point(binWidth_ * maxima[0].first, drawTarget_.rows),
                 BGR_MAGENTA);
    }

    // Draw the final chosen index if it's available (only going to be when
    // we're dumping the config)
    /*
    if (chosenMaximaIndex_ != -1) {
        int32_t max = maxima[chosenMaximaIndex_].first;
        cv::line(drawTarget_, cv::Point(binWidth_ * max, 0),
                 cv::Point(binWidth_ * max, drawTarget_.rows), BGR_RED);
    }
    */

    return drawTarget_;
}

// Finds the top 'N' maxima in the row being processed
IndexIntensityPairVec IntensityMap::sortedMaxima()
{
    // Find derivative of intensity curve
    /*
    cv::Mat_<double> sobelDerivatives;
    cv::Scharr(intensities_, sobelDerivatives, CV_64FC1, 1, 0, 1, 0,
               cv::BORDER_REPLICATE);
               */

    // Calculate derivatives with central difference
    cv::Mat_<double> sobelDerivatives(1, mapWidth_);
    for (int32_t i = 1; i < mapWidth_ - 1; ++i) {
        sobelDerivatives(i) = (intensities_(i + 1) - intensities_(i - 1)) / 2;
    }

    // Get indices of positive -> negative transitions, store in pairs (index,
    // intensity)
    IndexIntensityPairVec crossings;
    for (int32_t i = 0; i < sobelDerivatives.cols - 1; ++i) {
        if (sobelDerivatives(i) * sobelDerivatives(i + 1) <= 0 &&
            sobelDerivatives(i) > sobelDerivatives(i + 1)) {
            if (intensities_(i) >= intensities_(i + 1)) {
                crossings.emplace_back(i, intensities_(i));
            } else {
                crossings.emplace_back(i + 1, intensities_(i + 1));
            }
        }
    }

    // Filter out any crossings that are more than N voxels away from the center
    /*
    constexpr int32_t peakReadius = 3;
    crossings.erase(
        std::remove_if(std::begin(crossings), std::end(crossings),
                       [this, peakReadius](const IndexIntensityPair v) {
                           return std::abs(v.first - mapWidth_ / 2) >
                                  peakReadius;
                       }),
        std::end(crossings));
        */

    // Sort by distance from middle
    std::sort(
        std::begin(crossings), std::end(crossings),
        [this](IndexIntensityPair lhs, IndexIntensityPair rhs) {
            const int32_t distWeight = 85;
            const int32_t centerX = mapWidth_ / 2;
            const auto ldist = int32_t(std::abs(lhs.first - centerX));
            const auto rdist = int32_t(std::abs(rhs.first - centerX));
            const int32_t leftVal =
                distWeight * ldist +
                (100 - distWeight) * int32_t(std::round(-lhs.second * 100));
            const int32_t rightVal =
                distWeight * rdist +
                (100 - distWeight) * int32_t(std::round(-rhs.second * 100));
            return leftVal < rightVal;
        });

    return crossings;
}
