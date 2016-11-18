#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "apps/SliceImage.h"

namespace volcart
{

bool SliceImage::operator==(const SliceImage& b) const
{
    return (_w == b._w && _h == b._h && _depth == b._depth);
}

bool SliceImage::analyze()
{
    // return if the path is wrong or if this isn't a regular file
    if (!(boost::filesystem::exists(path)) ||
        !(boost::filesystem::is_regular_file(path)))
        return false;

    cv::Mat image;
    image = cv::imread(
        path.string(), CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

    _w = image.cols;
    _h = image.rows;

    _depth = image.depth();
    if (_depth != CV_16U)
        _convert = true;  // Convert if not a 16-bit, unsigned image

    cv::minMaxLoc(image, &_min, &_max);

    return true;
};

cv::Mat SliceImage::conformedImage()
{

    // Load the input
    cv::Mat input = cv::imread(
        path.string(), CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    cv::Mat output;

    // Remap 8 bit values to 16 bit
    if (input.depth() == CV_8U) {
        int minVal, maxVal;
        minVal = 0;
        maxVal = 255;
        input.convertTo(
            output, CV_16U, 65535.0 / (maxVal - minVal),
            -minVal * 65535.0 / (maxVal - minVal));
        // TODO: need to account for CV_8S and 32-bit images
    } else {
        input.copyTo(output);
    }

    // Convert colorspace to grayscale
    if (input.channels() > 1) {
        cv::cvtColor(
            output, output,
            CV_BGR2GRAY);  // OpenCV uses BGR to represent color image
    }

    return output;
};

}  // namespace volcart
