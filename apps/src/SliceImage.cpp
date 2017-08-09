#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "apps/SliceImage.hpp"

static const double MAX_16BPC = std::numeric_limits<uint16_t>::max();

namespace volcart
{

bool SliceImage::operator==(const SliceImage& b) const
{
    return (w_ == b.w_ && h_ == b.h_ && depth_ == b.depth_);
}

bool SliceImage::operator<(const SliceImage& b) const
{
    auto aName = boost::to_lower_copy<std::string>(path.filename().native());
    auto bName = boost::to_lower_copy<std::string>(b.path.filename().native());
    return aName < bName;
}

bool SliceImage::analyze()
{
    // return if the path is wrong or if this isn't a regular file
    if (!(boost::filesystem::exists(path)) ||
        !(boost::filesystem::is_regular_file(path)))
        return false;

    cv::Mat image;
    image =
        cv::imread(path.string(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

    w_ = image.cols;
    h_ = image.rows;

    depth_ = image.depth();
    if (image.depth() != CV_16U) {
        needsScale_ = true;
    }

    if (image.type() != CV_16UC1) {
        needsConvert_ = true;
    }

    cv::minMaxLoc(image, &min_, &max_);

    return true;
};

cv::Mat SliceImage::conformedImage()
{
    // Load the input
    cv::Mat input =
        cv::imread(path.string(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
    cv::Mat output;

    // Remap values to 16 bit
    if (needsScale_) {
        input.convertTo(
            output, CV_16U, MAX_16BPC / (max_ - min_),
            -min_ * MAX_16BPC / (max_ - min_));
        // TODO: #178
    } else {
        input.copyTo(output);
    }

    // Convert colorspace to grayscale
    if (input.channels() > 1) {
        cv::cvtColor(
            output, output,
            cv::COLOR_BGR2GRAY);  // OpenCV uses BGR to represent color image
    }

    return output;
};

}  // namespace volcart
