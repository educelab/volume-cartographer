#include "normalizedintensitymap.h"
#include "common.h"

using namespace volcart::segmentation;

NormalizedIntensityMap::NormalizedIntensityMap(cv::Mat r)
{
    cv::normalize(r, intensities_, 0, 1, CV_MINMAX, CV_64FC1);
    width_ = intensities_.cols;
    currentIntensity_ = intensities_(width_ / 2);
}

cv::Mat NormalizedIntensityMap::draw(const int32_t displayWidth,
                                     const int32_t displayHeight) const
{
    auto binWidth = cvRound(float(displayWidth) / width_);

    // Build intensity map
    cv::Mat mapImage(displayWidth, displayHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int32_t i = 1; i < width_; ++i) {
        auto p1 = cv::Point(
            binWidth * (i - 1),
            displayHeight - cvRound(displayHeight * intensities_(i - 1)));
        auto p2 =
            cv::Point(binWidth * (i),
                      displayHeight - cvRound(displayHeight * intensities_(i)));
        cv::line(mapImage, p1, p2, cv::Scalar(0, 255, 0));
    }

    // Sort by closest maxima available and draw line on that
    auto maxima = sortedMaxima();
    auto centerX = width_ / 2;

    // Vertical line at particle's current x position
    cv::line(mapImage, cv::Point(binWidth * centerX, 0),
             cv::Point(binWidth * centerX, mapImage.rows), BGR_YELLOW);

    // Line for position to be chosen
    for (const auto m : maxima) {
        cv::line(mapImage, cv::Point(binWidth * m.first, 0),
                 cv::Point(binWidth * m.first, mapImage.rows), BGR_BLUE);
    }

    return mapImage;
}

// Finds the top 'N' maxima in the row being processed
IndexIntensityPairVec NormalizedIntensityMap::sortedMaxima() const
{
    // Find derivative of intensity curve
    cv::Mat_<double> sobelDerivatives;
    cv::Sobel(intensities_, sobelDerivatives, CV_64FC1, 1, 0);

    // Get indices of positive -> negative transitions, store in pairs (index,
    // intensity)
    IndexIntensityPairVec crossings;
    for (int32_t i = 0; i < sobelDerivatives.cols - 1; ++i) {
        if (sobelDerivatives(i) * sobelDerivatives(i + 1) <= 1 &&
            sobelDerivatives(i) > sobelDerivatives(i + 1)) {
            if (intensities_(i) > intensities_(i + 1)) {
                crossings.emplace_back(i, intensities_(i));
            } else {
                crossings.emplace_back(i + 1, intensities_(i + 1));
            }
        }
    }

    // Filter out any crossings that are less than where this particle is now
    std::remove_if(crossings.begin(), crossings.end(),
                   [this](const IndexIntensityPair v) {
                       return v.second + 1e-2 < currentIntensity_ ||
                              v.second - 1e-2 < currentIntensity_;
                   });

    // Sort by distance from middle
    std::sort(crossings.begin(), crossings.end(),
              [this](IndexIntensityPair lhs, IndexIntensityPair rhs) {
                  /*
                  const double ldist =
                      2 * std::abs(lhs.first - center.x) / particleCount_;
                  const double rdist =
                      2 * std::abs(rhs.first - center.x) / particleCount_;
                  const int32_t distWeight = 75;
                  return (distWeight * ldist + (100 - distWeight) *
                  -lhs.second)
                  <
                         (distWeight * rdist + (100 - distWeight) *
                  -rhs.second);
                         */
                  const int32_t centerX = width_ / 2;
                  const auto ldist =
                      static_cast<int32_t>(std::abs(lhs.first - centerX));
                  const auto rdist =
                      static_cast<int32_t>(std::abs(rhs.first - centerX));
                  return ldist < rdist;
              });

    return crossings;
}
