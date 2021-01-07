#include "vc/core/util/ImageConversion.hpp"

namespace vc = volcart;

static inline cv::Mat CreateAlphaChannel(const cv::Size& size, int depth)
{
    // Create and scale the alpha channel
    cv::Mat alpha = cv::Mat::ones(size, depth);
    switch (alpha.depth()) {
        case CV_8U:
            alpha *= std::numeric_limits<uint8_t>::max();
            break;
        case CV_8S:
            alpha *= std::numeric_limits<int8_t>::max();
            break;
        case CV_16U:
            alpha *= std::numeric_limits<uint16_t>::max();
            break;
        case CV_16S:
            alpha *= std::numeric_limits<int16_t>::max();
            break;
        default:
            // do nothing
            break;
    }

    return alpha;
}

cv::Mat vc::QuantizeImage(const cv::Mat& m, int depth, bool scaleMinMax)
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
    double min{0};
    double max{1};
    if (scaleMinMax) {
        cv::minMaxLoc(output, &min, &max);
    } else {
        switch (m.depth()) {
            case CV_8U:
                max = std::numeric_limits<uint8_t>::max();
                break;
            case CV_8S:
                max = std::numeric_limits<int8_t>::max();
                break;
            case CV_16U:
                max = std::numeric_limits<uint16_t>::max();
                break;
            case CV_16S:
                max = std::numeric_limits<int16_t>::max();
                break;
            default:
                // do nothing
                break;
        }
    }
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

    // 1 -> 2
    else if (ic == 1 && oc == 2) {
        auto alpha = CreateAlphaChannel(m.size(), m.depth());
        cv::merge(std::vector<cv::Mat>{m, alpha}, output);
    }

    // 1 -> 4
    else if (ic == 1 && oc == 4) {
        cv::cvtColor(m, output, cv::COLOR_GRAY2BGRA);
    }

    // 2 -> 1
    else if (ic == 2 && oc == 1) {
        std::vector<cv::Mat> cns;
        cv::split(m, cns);
        output = cns[0];
    }

    // 2 -> 3
    else if (ic == 2 && oc == 3) {
        std::vector<cv::Mat> cns;
        cv::split(m, cns);
        cv::merge(std::vector<cv::Mat>{cns[0], cns[0], cns[0]}, output);
    }

    // 2 -> 4
    else if (ic == 2 && oc == 4) {
        std::vector<cv::Mat> cns;
        cv::split(m, cns);
        cv::merge(std::vector<cv::Mat>{cns[0], cns[0], cns[0], cns[1]}, output);
    }

    // 3 -> 1
    else if (ic == 3 && oc == 1) {
        cv::cvtColor(m, output, cv::COLOR_BGR2GRAY);
    }

    // 3 -> 2
    else if (ic == 3 && oc == 2) {
        // Convert color to gray
        cv::Mat gray;
        cv::cvtColor(m, gray, cv::COLOR_BGR2GRAY);

        // Add alpha
        auto alpha = CreateAlphaChannel(m.size(), m.depth());
        cv::merge(std::vector<cv::Mat>{gray, alpha}, output);
    }

    // 3 -> 4
    else if (ic == 3 && oc == 4) {
        cv::cvtColor(m, output, cv::COLOR_BGR2BGRA);
    }

    // 4 -> 1
    else if (ic == 4 && oc == 1) {
        cv::cvtColor(m, output, cv::COLOR_BGRA2GRAY);
    }

    // 4 -> 2
    else if (ic == 4 && oc == 2) {
        // Convert color to gray
        cv::Mat gray;
        cv::cvtColor(m, gray, cv::COLOR_BGRA2GRAY);

        // Extract alpha
        std::vector<cv::Mat> cns;
        cv::split(m, cns);

        // Merge gray + alpha
        cv::merge(std::vector<cv::Mat>{gray, cns[3]}, output);
    }

    // 4 -> 3
    else if (ic == 4 && oc == 3) {
        cv::cvtColor(m, output, cv::COLOR_BGRA2BGR);
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