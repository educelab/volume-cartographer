#include "normalizedintensitymap.h"
#include "common.h"

using namespace volcart::segmentation;

NormalizedIntensityMap::NormalizedIntensityMap(cv::Mat r)
{
    cv::normalize(r, _intensities, 0, 1, CV_MINMAX, CV_64FC1);
}

void NormalizedIntensityMap::draw(const int32_t displayWidth,
                                  const int32_t displayHeight) const
{
    auto binWidth = cvRound(float(displayWidth) / _intensities.cols);

    // Build intensity map
    cv::Mat mapImage(displayWidth, displayHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int32_t i = 1; i < _intensities.cols; ++i) {
        auto p1 = cv::Point(
            binWidth * (i - 1),
            displayHeight -
                cvRound(displayHeight * _intensities.at<double>(i - 1)));
        auto p2 =
            cv::Point(binWidth * (i),
                      displayHeight -
                          cvRound(displayHeight * _intensities.at<double>(i)));
        cv::line(mapImage, p1, p2, cv::Scalar(0, 255, 0));
    }

    // Sort by closest maxima available and draw line on that
    auto maxima = findMaxima();
    auto centerX = _intensities.cols / 2;
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

    cv::namedWindow("Intensity Map", cv::WINDOW_OPENGL);
    cv::imshow("Intensity Map", mapImage);
}

// Finds the top 'N' maxima in the row being processed
IndexDistPairVec NormalizedIntensityMap::findMaxima() const
{
    // Find derivative of intensity curve
    cv::Mat sobelDerivatives;
    cv::Sobel(_intensities, sobelDerivatives, CV_64FC1, 1, 0);

    // Get indices of positive -> negative transitions, store in pairs (index,
    // intensity)
    auto crossings = IndexDistPairVec();
    for (auto i = 0; i < sobelDerivatives.cols - 1; ++i) {
        if (sobelDerivatives.at<double>(i) *
                    sobelDerivatives.at<double>(i + 1) <=
                0 &&
            sobelDerivatives.at<double>(i) >
                sobelDerivatives.at<double>(i + 1)) {
            if (_intensities.at<double>(i) > _intensities.at<double>(i + 1)) {
                crossings.emplace_back(i, _intensities.at<double>(i));
            } else {
                crossings.emplace_back(i + 1, _intensities.at<double>(i + 1));
            }
        }
    }

    return crossings;
}
