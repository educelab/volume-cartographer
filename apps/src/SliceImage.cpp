#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "apps/SliceImage.hpp"
#include "vc/core/io/FileExtensionFilter.hpp"

static const double MAX_16BPC = std::numeric_limits<uint16_t>::max();

namespace volcart
{

bool SliceImage::operator==(const SliceImage& b) const
{
    return (w_ == b.w_ && h_ == b.h_ && type_ == b.type_);
}

bool SliceImage::operator<(const SliceImage& b) const
{
    auto aName = boost::to_lower_copy<std::string>(path.filename().native());
    auto bName = boost::to_lower_copy<std::string>(b.path.filename().native());

    // Lexicographical sort if paths are the same length
    if (aName.size() == bName.size()) {
        return aName < bName;
    }

    // Otherwise, compare only the size
    // This is a hack to deal with files with non-leading zeros
    return aName.size() < bName.size();
}

bool SliceImage::analyze()
{
    // return if the path is wrong or if this isn't a regular file
    if (!(boost::filesystem::exists(path)) ||
        !(boost::filesystem::is_regular_file(path)))
        return false;

    // Set needsConvert_ if it's not a tif
    needsConvert_ = !io::FileExtensionFilter(path, {"tif", "tiff"});

    auto image =
        cv::imread(path.string(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

    w_ = image.cols;
    h_ = image.rows;

    type_ = image.type();
    if (type_ != CV_16UC1) {
        needsConvert_ = true;
    }

    if (image.depth() != CV_16U) {
        needsScale_ = true;
    }

    cv::minMaxLoc(image, &min_, &max_);

    return true;
}

cv::Mat SliceImage::conformedImage()
{
    // Load the image
    auto image =
        cv::imread(path.string(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

    // Remap values to 16 bit
    if (needsScale_) {
        image.convertTo(
            image, CV_16U, MAX_16BPC / (max_ - min_),
            -min_ * MAX_16BPC / (max_ - min_));
    }

    // Convert colorspace to grayscale
    if (image.channels() > 1) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }

    return image;
}

}  // namespace volcart
