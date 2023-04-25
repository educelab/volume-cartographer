#include "vc/core/util/ApplyLUT.hpp"

#include <opencv2/imgproc.hpp>

#include "vc/core/util/ImageConversion.hpp"
#include "vc/core/util/Iteration.hpp"

using namespace volcart;
namespace vc = volcart;

// Calculate the LUT bin a value belongs in
static int ValueToBin(float value, float min, float max, int bins)
{
    // End points are easy
    if (value == max) {
        return bins - 1;
    }

    if (value == min) {
        return 0;
    }

    // Calculate intermediate point
    auto pos = (value - min) / (max - min);

    // Clamp to [0, 1]
    pos = std::max(std::min(pos, 1.F), 0.F);

    auto bin = std::round(float(bins - 1) * pos);

    return static_cast<int>(bin);
}

cv::Mat vc::ApplyLUT(
    const cv::Mat& img, const cv::Mat& lut, float min, float max, bool invert)
{
    // Validate the LUT
    if (lut.depth() != CV_8U) {
        throw std::invalid_argument("LUT must be 8bpc");
    }
    if (lut.rows != 1) {
        throw std::invalid_argument("LUT must have shape (1, bins)");
    }
    if (lut.channels() != 1 and lut.channels() != 3) {
        throw std::invalid_argument("LUT must be gray/RGB");
    }

    // Convert input img to 32FC1
    cv::Mat gray;
    if (img.channels() != 1) {
        gray = ColorConvertImage(img);
    } else {
        gray = img.clone();
    }
    gray.convertTo(gray, CV_32F);

    // Construct output image
    cv::Mat output = cv::Mat::zeros(gray.rows, gray.cols, lut.type());

    // Iterate over pixels
    for (const auto& [y, x] : range2D(gray.rows, gray.cols)) {
        auto val = gray.at<float>(y, x);
        auto bin = ValueToBin(val, min, max, lut.cols);

        // Invert bin assignment values
        if (invert) {
            bin = lut.cols - 1 - bin;
        }

        // Assign to image
        if (lut.channels() == 1) {
            output.at<uint8_t>(y, x) = lut.at<uint8_t>(0, bin);
        } else if (lut.channels() == 3) {
            output.at<cv::Vec3b>(y, x) = lut.at<cv::Vec3b>(0, bin);
        }
    }

    return output;
}

cv::Mat vc::ApplyLUT(
    const cv::Mat& img,
    const cv::Mat& lut,
    float min,
    float mid,
    float max,
    bool invert)
{
    // Validate the LUT
    if (lut.depth() != CV_8U) {
        throw std::invalid_argument("LUT must be 8bpc");
    }
    if (lut.rows != 1) {
        throw std::invalid_argument("LUT must have shape (1, bins)");
    }
    if (lut.channels() != 1 and lut.channels() != 3) {
        throw std::invalid_argument("LUT must be gray/RGB");
    }

    // Convert input img to 32FC1
    cv::Mat gray;
    if (img.channels() != 1) {
        gray = ColorConvertImage(img);
    } else {
        gray = img.clone();
    }
    gray.convertTo(gray, CV_32F);

    // Construct output image
    cv::Mat output = cv::Mat::zeros(gray.rows, gray.cols, lut.type());

    // Mid point bin
    auto midBin = static_cast<int>(std::round(lut.cols / 2));

    // Iterate over pixels
    for (const auto& [y, x] : range2D(gray.rows, gray.cols)) {
        auto val = gray.at<float>(y, x);
        int bin{0};
        if (val == mid) {
            bin = midBin;
        } else if (val < mid) {
            bin = ValueToBin(val, min, mid, midBin);
        } else {
            bin = ValueToBin(val, mid, max, lut.cols - midBin) + midBin;
        }

        // Invert bin assignment values
        if (invert) {
            bin = lut.cols - 1 - bin;
        }

        // Assign to image
        if (lut.channels() == 1) {
            output.at<uint8_t>(y, x) = lut.at<uint8_t>(0, bin);
        } else if (lut.channels() == 3) {
            output.at<cv::Vec3b>(y, x) = lut.at<cv::Vec3b>(0, bin);
        }
    }

    return output;
}

cv::Mat vc::ApplyLUT(
    const cv::Mat& img, const cv::Mat& lut, bool invert, const cv::Mat& mask)
{
    // Default min/max
    double min{0};
    double max{1};

    // Get min/max
    cv::minMaxIdx(img, &min, &max, nullptr, nullptr, mask);

    // Apply the LUT
    return ApplyLUT(
        img, lut, static_cast<float>(min), static_cast<float>(max), invert);
}

cv::Mat vc::GenerateLUTScaleBar(
    const cv::Mat& lut, bool invert, size_t height, size_t width)
{
    // Construct a single row
    auto maxDim = static_cast<int>(std::max(width, height));
    cv::Mat row = cv::Mat(1, maxDim, CV_32FC1);
    auto delta = 1.F / static_cast<float>(maxDim - 1);
    for (const auto& px : range(maxDim)) {
        row.at<float>(px) = delta * static_cast<float>(px);
    }

    // Rotate if vertical
    if (height > width) {
        cv::rotate(row, row, cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    // Expand to 2D
    cv::Mat bar;
    auto w = static_cast<int>(width);
    auto h = static_cast<int>(height);
    cv::resize(row, bar, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);

    // Apply color map
    return volcart::ApplyLUT(bar, lut, invert);
}