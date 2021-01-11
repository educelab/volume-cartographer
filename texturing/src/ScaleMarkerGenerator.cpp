#include "vc/texturing/ScaleMarkerGenerator.hpp"

#include <iostream>

#include <opencv2/imgproc.hpp>

#include "vc/core/util/ImageConversion.hpp"

namespace vc = volcart;
namespace vct = volcart::texturing;

static constexpr double MICRON_TO_INCH = 0.0000393701;
static constexpr double MICRON_TO_CM = 0.0001;
static constexpr int THICKNESS = 2;
static constexpr int LINE_TYPE = cv::LINE_8;
static constexpr int FONT_TYPE = cv::FONT_HERSHEY_PLAIN;
static constexpr int TICK_BEGIN_X = 0;
static constexpr int TICK_BEGIN_Y = 0;
static constexpr int MAJOR_TICK_END_X = 120;
static constexpr int MINOR_TICK_END_X = 80;
static constexpr int TEXT_OFFSET_X = 10;
static constexpr int TEXT_OFFSET_Y = 5;
static constexpr int SCALE_WIDTH = 200;

cv::Mat vct::ScaleMarkerGenerator::compute()
{
    // Safety check
    if (inputImg_.empty()) {
        throw std::domain_error("Empty input image");
    }

    switch (type_) {
        case Type::Imperial:
            scaleFactor_ = MICRON_TO_INCH;
            scaleUnit_ = "in";
            scaleImg_ = generate_scale_bar_();
            break;
        case Type::Metric:
            scaleFactor_ = MICRON_TO_CM;
            scaleUnit_ = "cm";
            scaleImg_ = generate_scale_bar_();
            break;
        case Type::ReferenceImage:
            scaleFactor_ = refImgPixSize_ / inputImgPixSize_;
            scaleImg_ = resize_ref_image_();
            break;
    }

    // Copy the input image and convert to 8U
    auto inputCopy = inputImg_.clone();
    inputCopy = vc::QuantizeImage(inputCopy, CV_8U);
    // Convert to color only if grayscale
    if (inputCopy.channels() == 1) {
        cv::cvtColor(inputCopy, inputCopy, cv::COLOR_GRAY2BGR);
    } else if (inputCopy.channels() != 3) {
        auto msg = "Unsupported number of channels: " +
                   std::to_string(inputCopy.channels());
        throw std::domain_error(msg);
    }

    // Setup final image
    auto outputWidth = inputCopy.cols + scaleImg_.cols;
    outputImg_ = cv::Mat::zeros(inputCopy.rows, outputWidth, CV_8UC3);

    // Copy scale image into left ROI
    cv::Rect leftROI(0, 0, scaleImg_.cols, scaleImg_.rows);
    cv::Mat left(outputImg_, leftROI);
    scaleImg_.copyTo(left);

    // Copy input image into right ROI
    cv::Rect rightROI(scaleImg_.cols, 0, inputCopy.cols, inputCopy.rows);
    cv::Mat right(outputImg_, rightROI);
    inputCopy.copyTo(right);

    return outputImg_;
}

cv::Mat vct::ScaleMarkerGenerator::resize_ref_image_()
{
    // resizing the sample character
    cv::Mat resized;
    cv::resize(refImg_, resized, cv::Size(), scaleFactor_, scaleFactor_);
    // convert sample character to correct number of channels
    resized = vc::QuantizeImage(resized, CV_8U);
    if (resized.channels() == 1) {
        cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);
    } else if (resized.channels() != 3) {
        auto msg = "Unsupported number of channels: " +
                   std::to_string(resized.channels());
        throw std::domain_error(msg);
    }
    return resized;
}

cv::Mat vct::ScaleMarkerGenerator::generate_scale_bar_()
{
    // Setup empty output image
    cv::Mat barImg = cv::Mat::zeros(inputImg_.rows, SCALE_WIDTH, CV_8UC3);

    // calculate the scale length
    auto inputHeight = inputImg_.rows;
    auto scaleMicrons = inputImgPixSize_ * inputHeight;
    auto scaleLength = scaleMicrons * scaleFactor_;

    // Setup tick position variables
    const int numMajorTicks = static_cast<int>(scaleLength) + 1;
    const auto majorTickDistY = static_cast<float>(inputHeight / scaleLength);
    const int numMinorIntervals = 10;
    const auto minorTickDistY = majorTickDistY / numMinorIntervals;
    float majorYPosition = 0;
    float minorYPosition = 0;
    cv::Point2f majorStart{TICK_BEGIN_X, TICK_BEGIN_Y};
    cv::Point2f majorEnd{MAJOR_TICK_END_X, TICK_BEGIN_Y};
    cv::Point2f minorStart{TICK_BEGIN_X, TICK_BEGIN_Y};
    cv::Point2f minorEnd{MINOR_TICK_END_X, TICK_BEGIN_Y};
    cv::Point2f textPos{
        MAJOR_TICK_END_X + TEXT_OFFSET_X, TICK_BEGIN_Y + TEXT_OFFSET_Y};

    // Generate the scale bar
    for (int i = 0; i < numMajorTicks; i++) {
        // Update y position
        majorYPosition = TICK_BEGIN_Y + (i * majorTickDistY);
        minorYPosition = majorYPosition;

        // draw the major unit line
        majorStart.y = majorEnd.y = majorYPosition;
        cv::line(
            barImg, majorStart, majorEnd, scaleBarColor_, THICKNESS, LINE_TYPE);

        // generate the major unit label
        textPos.x = MAJOR_TICK_END_X + TEXT_OFFSET_X;
        textPos.y = majorYPosition + TEXT_OFFSET_Y;
        draw_tick_label_(barImg, std::to_string(i), textPos);

        // generate the minor unit lines
        for (int j = 0; j < numMinorIntervals; j++) {
            minorStart.y = minorEnd.y = minorYPosition;
            cv::line(
                barImg, minorStart, minorEnd, scaleBarColor_, THICKNESS,
                LINE_TYPE);
            minorYPosition += minorTickDistY;

            // If we don't have a lot of major ticks, add labels for half ticks
            if (numMajorTicks <= 2 && j == 4) {
                textPos.x = MINOR_TICK_END_X + TEXT_OFFSET_X;
                textPos.y = minorYPosition + TEXT_OFFSET_Y;
                auto v = std::to_string(i) + ".5";
                draw_tick_label_(barImg, v, textPos);
            }
        }
    }

    return barImg;
}

void vct::ScaleMarkerGenerator::draw_tick_label_(
    cv::Mat img, const std::string& value, const cv::Point& position)
{
    auto indexString = value + " " + scaleUnit_;
    cv::putText(
        img, indexString, position, FONT_TYPE, 1, scaleBarColor_, THICKNESS,
        LINE_TYPE);
}
