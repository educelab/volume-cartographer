#include "vc/core/util/ImageConversion.hpp"

namespace vc = volcart;

cv::Mat vc::QuantizeImage(const cv::Mat& m, int depth)
{
    // Make sure we have work to do
    if (m.depth() == depth) {
        // Depth already matches. Do nothing.
        return m;
    }

    // Copy to output
    auto output = m.clone();

    // Setup the max value for integer images
    double outputMax{1.0};
    switch (depth) {
        case CV_8U:
            outputMax = std::numeric_limits<uint8_t>::max();
            break;
        case CV_8S:
            outputMax = std::numeric_limits<int8_t>::max();
            break;
        case CV_16U:
            outputMax = std::numeric_limits<uint16_t>::max();
            break;
        case CV_16S:
            outputMax = std::numeric_limits<int16_t>::max();
            break;
        default:
            // do nothing
            break;
    }

    // Scale to output
    double min, max;
    cv::minMaxLoc(output, &min, &max);
    output.convertTo(
        output, depth, outputMax / (max - min), -min * outputMax / (max - min));

    return output;
}

cv::Mat vc::ColorConvertImage(const cv::Mat& m, int channels)
{
    // Make sure we have work to do
    if (m.channels() == channels) {
        // Do nothing. Already correct.
        return m;
    }

    // short vars for convenience
    auto ic = m.channels();
    auto oc = channels;

    // Setup the output
    cv::Mat output;

    // 1 -> 3
    if (ic == 1 && oc == 3) {
        cv::cvtColor(m, output, cv::COLOR_GRAY2BGR);
    }

    // 1 -> 4
    else if (ic == 1 && oc == 4) {
        cv::cvtColor(m, output, cv::COLOR_GRAY2BGRA);
    }

    // 2 -> 3
    else if (ic == 2 && oc == 3) {
        cv::Mat cns[2];
        cv::split(m, cns);
        cv::merge(std::vector<cv::Mat>{cns[0], cns[0], cns[0]}, output);
    }

    // 2 -> 4
    else if (ic == 2 && oc == 4) {
        cv::Mat cns[2];
        cv::split(m, cns);
        cv::merge(std::vector<cv::Mat>{cns[0], cns[0], cns[0], cns[1]}, output);
    }

    // 3 -> 1
    else if (ic == 3 && oc == 1) {
        cv::cvtColor(m, output, cv::COLOR_BGR2GRAY);
    }

    // 3 -> 4
    else if (ic == 3 && oc == 4) {
        cv::cvtColor(m, output, cv::COLOR_BGR2BGRA);
    }

    // 4 -> 3
    else if (ic == 4 && oc == 3) {
        cv::cvtColor(m, output, cv::COLOR_BGRA2BGR);
    }

    // 4 -> 1
    else if (ic == 4 && oc == 1) {
        cv::cvtColor(m, output, cv::COLOR_BGRA2GRAY);
    }

    // unknown conversion
    else {
        auto msg = "Cannot convert image with " + std::to_string(ic) +
                   " channels to image with " + std::to_string(oc) +
                   " channels.";
        throw std::runtime_error(msg);
    }

    return output;
}