#include "NormalizedIntensityMap.h"

using namespace volcart::segmentation;

NormalizedIntensityMap::NormalizedIntensityMap(cv::Mat r) {
    cv::normalize(r, _intensities, 0, 1, CV_MINMAX, CV_64FC1);
}

void NormalizedIntensityMap::draw(const uint32_t displayWidth, const uint32_t displayHeight) const {
    int32_t binWidth = cvRound(float(displayWidth) / _intensities.cols);

    // Build intensity map
    cv::Mat mapImage(displayWidth, displayHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int32_t i = 1; i < _intensities.cols; ++i) {
        auto p1 = cv::Point(binWidth * (i-1), cvRound(displayHeight * _intensities.at<double>(i-1)));
        auto p2 = cv::Point(binWidth * (i)  , cvRound(displayHeight * _intensities.at<double>(i)));
        cv::line(mapImage, p1, p2, cv::Scalar(0, 255, 0));
    }

    cv::namedWindow("Intensity Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Intensity Map", mapImage);
}

// Finds the top 'N' maxima in the row being processed
std::vector<std::pair<cv::Point, double>> NormalizedIntensityMap::findNMaxima(const uint32_t n) const {
    cv::Mat mask = cv::Mat::ones(_intensities.rows, _intensities.cols, CV_8UC1);
    auto pairs = std::vector<std::pair<cv::Point, double>>();
    for (uint32_t i = 0; i < n; ++i) {
        double maxVal = 0.0;
        cv::Point maxLoc;
        cv::minMaxLoc(_intensities, nullptr, &maxVal, nullptr, &maxLoc, mask);
        std::cout << "point = " << maxLoc << std::endl;
        pairs.push_back(std::make_pair(maxLoc, maxVal));
        mask.at<uint8_t>(0, maxLoc.y) = 0;
    }
    return pairs;
}

cv::Mat NormalizedIntensityMap::intensities() const {
    return _intensities;
}