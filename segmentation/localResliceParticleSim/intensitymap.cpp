#include "intensitymap.h"
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

    // Sort by closest maxima available and draw line on that
    auto maxima = sortedMaxima();
    // auto centerX = mapWidth_ / 2;

    // A line for each candidate position
    for (const auto m : maxima) {
        cv::line(drawTarget_, cv::Point(binWidth_ * m.first, 0),
                 cv::Point(binWidth_ * m.first, drawTarget_.rows), BGR_BLUE);
    }

    // A line at particle's current x position
    /*
    cv::line(drawTarget_, cv::Point(binWidth_ * centerX, 0),
             cv::Point(binWidth_ * centerX, drawTarget_.rows), BGR_YELLOW);
             */

    // A Line for top maxima
    cv::line(drawTarget_, cv::Point(binWidth_ * maxima[0].first, 0),
             cv::Point(binWidth_ * maxima[0].first, drawTarget_.rows),
             BGR_MAGENTA);

    // Draw the final chosen index if it's available (only going to be when
    // we're dumping the config)
    if (chosenMaximaIndex_ != -1) {
        int32_t max = maxima[chosenMaximaIndex_].first;
        cv::line(drawTarget_, cv::Point(binWidth_ * max, 0),
                 cv::Point(binWidth_ * max, drawTarget_.rows), BGR_RED);
    }

    return drawTarget_;
}

// Finds the top 'N' maxima in the row being processed
IndexIntensityPairVec IntensityMap::sortedMaxima() const
{
    // Find derivative of intensity curve
    cv::Mat_<double> sobelDerivatives;
    cv::Sobel(intensities_, sobelDerivatives, CV_64FC1, 1, 0);
    int32_t mid = mapWidth_ / 2;
    std::cout << sobelDerivatives(mid - 1) << ", " << sobelDerivatives(mid)
              << ", " << sobelDerivatives(mid + 1) << std::endl;
    std::cout << intensities_(mid - 1) << ", " << intensities_(mid) << ", "
              << intensities_(mid + 1) << std::endl;

    // Get indices of positive -> negative transitions, store in pairs (index,
    // intensity)
    IndexIntensityPairVec crossings;
    for (int32_t i = 0; i < sobelDerivatives.cols - 1; ++i) {
        if (sobelDerivatives(i) * sobelDerivatives(i + 1) <= 0 &&
            sobelDerivatives(i) > sobelDerivatives(i + 1)) {
            if (intensities_(i) > intensities_(i + 1)) {
                crossings.emplace_back(i, intensities_(i));
            } else {
                crossings.emplace_back(i + 1, intensities_(i + 1));
            }
        }
    }

    // Filter out any crossings that are less than where this particle is now
    /*
    std::remove_if(crossings.begin(), crossings.end(),
                   [this](const IndexIntensityPair v) {
                       return v.second + 1e-2 < currentIntensity_ ||
                              v.second - 1e-2 < currentIntensity_;
                   });
                   */

    // Sort by distance from middle
    std::sort(crossings.begin(), crossings.end(),
              [this](IndexIntensityPair lhs, IndexIntensityPair rhs) {
                  /*
                  const double ldist =
                      2 * std::abs(lhs.first - center.x) / particleCount_;
                  const double rdist =
                      2 * std::abs(rhs.first - center.x) / particleCount_;
                  return (distWeight * ldist + (100 - distWeight) *
                  -lhs.second)
                  <
                         (distWeight * rdist + (100 - distWeight) *
                  -rhs.second);
                         */
                  const int32_t distWeight = 75;
                  const int32_t centerX = mapWidth_ / 2;
                  const auto ldist = int32_t(std::abs(lhs.first - centerX));
                  const auto rdist = int32_t(std::abs(rhs.first - centerX));
                  const int32_t leftVal =
                      distWeight * ldist +
                      int32_t((100 - distWeight) * -lhs.second * 100);
                  const int32_t rightVal =
                      distWeight * rdist +
                      int32_t((100 - distWeight) * -rhs.second * 100);
                  return leftVal < rightVal;
              });

    return crossings;
}
