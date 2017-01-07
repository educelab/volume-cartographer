#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "segmentation/lrps/IntensityMap.h"

using namespace volcart::segmentation;

IntensityMap::IntensityMap(
    cv::Mat r,
    int32_t stepSize,
    int32_t peakDistanceWeight,
    bool shouldIncludeMiddle)
    : stepSize_(stepSize)
    , peakDistanceWeight_(peakDistanceWeight)
    , displayWidth_(200)
    , displayHeight_(200)
    , drawTarget_(displayWidth_, displayHeight_, CV_8UC3, BGR_BLACK)
    , chosenMaximaIndex_(-1)
    , shouldIncludeMiddle_(shouldIncludeMiddle)
{
    // DEBUG - need to convert to 8 bit before we can do anything
    r.convertTo(r, CV_8UC1, 1.0 / std::numeric_limits<uint8_t>::max());
    cv::equalizeHist(r, r);
    resliceData_ = r;

    cv::Mat normReslice(r.rows, r.cols, CV_64F);
    cv::normalize(r, normReslice, 0, 1, cv::NORM_MINMAX, CV_64F);
    intensities_ = normReslice.row(normReslice.rows / 2 + stepSize);
    mapWidth_ = intensities_.cols;
    binWidth_ = cvRound(float(displayWidth_) / mapWidth_);
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
        cv::circle(
            drawTarget_,
            {binWidth_ * i,
             displayHeight_ - cvRound(displayWidth_ * intensities_(i))},
            2, pointColor);
        cv::line(drawTarget_, p1, p2, BGR_GREEN);
    }
    auto maxima = sortedMaxima();

    // A line for each candidate position
    for (const auto m : maxima) {
        cv::line(
            drawTarget_, cv::Point(binWidth_ * m.first, 0),
            cv::Point(binWidth_ * m.first, drawTarget_.rows), BGR_BLUE);
    }

    // A Line for top maxima
    if (!maxima.empty()) {
        cv::line(
            drawTarget_, cv::Point(binWidth_ * maxima[0].first, 0),
            cv::Point(binWidth_ * maxima[0].first, drawTarget_.rows),
            BGR_MAGENTA);
    }

    // Draw the final chosen index if it's available (only going to be when
    // we're dumping the config)
    if (chosenMaximaIndex_ != -1) {
        int32_t max = maxima[chosenMaximaIndex_].first;
        cv::line(
            drawTarget_, cv::Point(binWidth_ * max, 0),
            cv::Point(binWidth_ * max, drawTarget_.rows), BGR_RED);
    }

    return drawTarget_;
}

// Finds the top 'N' maxima in the row being processed
std::deque<std::pair<int32_t, double>> IntensityMap::sortedMaxima()
{
    bool includesMiddle = false;
    std::deque<std::pair<int32_t, double>> crossings;
    for (int32_t i = 1; i < intensities_.cols - 1; ++i) {
        if (intensities_(i) >= intensities_(i - 1) &&
            intensities_(i) >= intensities_(i + 1)) {
            crossings.emplace_back(i, intensities_(i));
            if (i == intensities_.cols / 2) {
                includesMiddle = true;
            }
        }
    }

    // Filter out any crossings that are more than N voxels away from the center
    crossings.erase(
        std::remove_if(
            std::begin(crossings), std::end(crossings),
            [this](auto v) {
                return std::abs(v.first - mapWidth_ / 2) > peakRadius_;
            }),
        std::end(crossings));

    // Sort by distance from middle
    std::sort(
        std::begin(crossings), std::end(crossings), [this](auto lhs, auto rhs) {
            const int32_t centerX = resliceData_.cols / 2;
            const auto ldist = std::sqrt(
                (lhs.first - centerX) * (lhs.first - centerX) +
                stepSize_ * stepSize_);
            const auto rdist = std::sqrt(
                (rhs.first - centerX) * (rhs.first - centerX) +
                stepSize_ * stepSize_);

            // The 2 * peakRadius comes from the fact that the {rhs,lhs}.second
            // is in the range [0, 1]. So we scale this to the same range that
            // ldist and rdist will be, which is exactly [0, 2 * peakRadius]
            // because of the above filtering.
            const int32_t leftVal = std::round(peakDistanceWeight_ * ldist) +
                                    std::round(
                                        (100 - peakDistanceWeight_) *
                                        -lhs.second * 2 * peakRadius_);
            const int32_t rightVal = std::round(peakDistanceWeight_ * rdist) +
                                     std::round(
                                         (100 - peakDistanceWeight_) *
                                         -rhs.second * 2 * peakRadius_);
            return leftVal < rightVal;
        });

    // Append a "going straight down" option so that it exists in the list of
    // possible choices
    if (!includesMiddle && shouldIncludeMiddle_) {
        auto mid = intensities_.cols / 2;
        crossings.emplace_back(mid, intensities_(mid));
    }

    return crossings;
}
