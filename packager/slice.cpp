//
// Created by Seth Parker on 7/31/15.
//

#include "slice.h"

namespace volcart {

bool Slice::operator==(const Slice& b) const {
    return (_w == b._w && _h == b._h && _depth == b._depth);
}

bool Slice::analyze() {
    // return if the path is wrong or if this isn't a regular file
    if (!(boost::filesystem::exists(path)) || !(boost::filesystem::is_regular_file(path))) return false;

    cv::Mat image;
    image = cv::imread( path.string(), CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );

    _w = image.cols;
    _h = image.rows;

    //The depth of the image as OpenCV int ID
    // 0 = CV_8U  - 8-bit unsigned
    // 1 = CV_8S  - 8-bit signed
    // 2 = CV_16U - 16-bit unsigned
    // 3 = CV_16S - 16-bit signed
    // 4 = CV_32S - 32-bit signed integers
    // 5 = CV_32F - 32-bit floating-point
    // 6 = CV_64F - 64-bit floating-point
    _depth = image.depth();
    if (_depth != 2) _convert = true; // We can only handle CV_16U files internally

    cv::minMaxLoc(image, &_min, &_max);

    return true;
};



} // namespace volcart