#include "NormalizedIntensityMap.h"

using namespace volcart::segmentation;

const auto BGR_RED    = cv::Scalar(0, 0   , 0xFF);
const auto BGR_YELLOW = cv::Scalar(0, 0xFF, 0xFF);

NormalizedIntensityMap::NormalizedIntensityMap(cv::Mat r) {
    cv::normalize(r, _intensities, 0, 1, CV_MINMAX, CV_64FC1);
}

void NormalizedIntensityMap::draw(const uint32_t displayWidth, const uint32_t displayHeight) const {
    int32_t binWidth = cvRound(float(displayWidth) / _intensities.cols);

    // Build intensity map
    cv::Mat mapImage(displayWidth, displayHeight, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int32_t i = 1; i < _intensities.cols; ++i) {
        auto p1 = cv::Point(binWidth * (i-1), displayHeight - cvRound(displayHeight * _intensities.at<double>(i-1)));
        auto p2 = cv::Point(binWidth * (i)  , displayHeight - cvRound(displayHeight * _intensities.at<double>(i)));
        cv::line(mapImage, p1, p2, cv::Scalar(0, 255, 0));
    }

    // DEBUG: draw vertical lines at maxima points
    auto maxima = findNMaxima();
    for (auto m : maxima) {
        cv::line(mapImage, cv::Point(binWidth * m.first, 0), cv::Point(binWidth * m.first, mapImage.rows), BGR_RED);
    }

    // Vertical line at particle's current x position
    auto centerX = _intensities.cols / 2;
    cv::line(mapImage, cv::Point(binWidth * centerX, 0), cv::Point(binWidth * centerX, mapImage.rows), BGR_YELLOW);

    cv::namedWindow("Intensity Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Intensity Map", mapImage);
}

// Finds the top 'N' maxima in the row being processed
std::vector<std::pair<uint32_t, double>> NormalizedIntensityMap::findNMaxima(const uint32_t n) const {
    using Pair = std::pair<uint32_t, double>;
    // Find derivative of intensity curve
    cv::Mat sobelDerivatives;
    cv::Sobel(_intensities, sobelDerivatives, CV_64FC1, 1, 0);

    // Get indices of positive -> negative/negative -> positive transitions, store in pairs (index, intensity), then
    // sort in reverse.
    auto crossings = std::vector<Pair>();
    for (auto i = 0; i < sobelDerivatives.cols - 1; ++i) {
        if (sobelDerivatives.at<double>(i) * sobelDerivatives.at<double>(i+1) < 0) {
            if (_intensities.at<double>(i) > _intensities.at<double>(i+1)) {
                crossings.push_back(std::make_pair(i, _intensities.at<double>(i)));
            } else {
                crossings.push_back(std::make_pair(i+1, _intensities.at<double>(i+1)));
            }
        }
    }
    std::sort(crossings.begin(), crossings.end(), [](Pair lhs, Pair rhs) { return lhs.second > rhs.second; });
    for (auto c : crossings) {
        std::cout << "(" << c.first << ", " << c.second << ") ";
    }
    std::cout << std::endl;

    // Take first (up to) N points
    auto nPoints = std::vector<Pair>();
    std::cout << "Crossings length: " << crossings.size() << std::endl;
    std::copy_n(crossings.begin(), std::min({n, uint32_t(crossings.size())}), std::back_inserter(nPoints));
    return nPoints;
}