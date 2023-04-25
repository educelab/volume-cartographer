#include <opencv2/imgproc.hpp>

#include "vc/core/util/Canny.hpp"

namespace vc = volcart;

auto vc::Canny(const cv::Mat src, const vc::CannySettings settings) -> cv::Mat
{
    int gaussianKernel = 2 * settings.blurSize + 1;
    int aperture = 2 * settings.apertureSize + 3;
    cv::Mat canny;
    cv::GaussianBlur(src, canny, {gaussianKernel, gaussianKernel}, 0);
    if (settings.bilateral) {
        cv::bilateralFilter(canny.clone(), canny, gaussianKernel, 75, 75);
    }

    // Run Canny Edge Detect
    cv::Canny(
        canny, canny, settings.minThreshold, settings.maxThreshold, aperture,
        true);

    // Apply mask
    if (!settings.mask.empty()) {
        canny.copyTo(canny, settings.mask);
    }

    // Replace canny edges with contour edges
    if (settings.contour) {
        int closingKernel = 2 * settings.closingSize + 1;
        // Apply closing to fill holes and gaps.
        cv::Mat kernel = cv::Mat::ones(closingKernel, closingKernel, CV_8UC1);
        cv::morphologyEx(canny, canny, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(
            canny, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        canny = cv::Mat::zeros(src.size(), CV_8UC1);
        cv::drawContours(canny, contours, -1, {255});
    }

    return canny;
}