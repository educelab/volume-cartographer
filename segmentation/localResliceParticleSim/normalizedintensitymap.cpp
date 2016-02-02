#include "normalizedintensitymap.h"
#include "common.h"

using namespace volcart::segmentation;

NormalizedIntensityMap::NormalizedIntensityMap(cv::Mat r)
{
    cv::normalize(r, intensities_, 0, 1, CV_MINMAX, CV_64FC1);
}

void NormalizedIntensityMap::draw(const int32_t displayWidth,
                                  const int32_t displayHeight) const
{
    auto binWidth = cvRound(float(displayWidth) / intensities_.cols);

    // Build intensity map
    cv::Mat mapImage(displayWidth, displayHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int32_t i = 1; i < intensities_.cols; ++i) {
        auto p1 = cv::Point(
            binWidth * (i - 1),
            displayHeight - cvRound(displayHeight * intensities_(i - 1)));
        auto p2 =
            cv::Point(binWidth * (i),
                      displayHeight - cvRound(displayHeight * intensities_(i)));
        cv::line(mapImage, p1, p2, cv::Scalar(0, 255, 0));
    }

    // Sort by closest maxima available and draw line on that
    auto maxima = findMaxima();
    auto centerX = intensities_.cols / 2;
    auto minDist = std::numeric_limits<int32_t>::max();
    auto minIdx = -1;
    for (size_t i = 0; i < maxima.size(); ++i) {
        auto currentDist = std::abs(int32_t(maxima[i].first - centerX));
        if (currentDist < minDist) {
            minDist = currentDist;
            minIdx = i;
        }
    }
    cv::line(mapImage, cv::Point(binWidth * maxima[minIdx].first, 0),
             cv::Point(binWidth * maxima[minIdx].first, mapImage.rows),
             BGR_BLUE);

    // Vertical line at particle's current x position
    cv::line(mapImage, cv::Point(binWidth * centerX, 0),
             cv::Point(binWidth * centerX, mapImage.rows), BGR_YELLOW);

    cv::namedWindow("Intensity Map", cv::WINDOW_NORMAL);
    cv::imshow("Intensity Map", mapImage);
}

// Finds the top 'N' maxima in the row being processed
IndexIntensityPairVec NormalizedIntensityMap::findMaxima() const
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

    return crossings;
}
