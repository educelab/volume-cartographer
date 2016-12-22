#include <opencv2/imgproc.hpp>

#include "common/types/Slice.h"

#define BGR_MAGENTA cv::Scalar(0xFF, 0, 0xFF)

Slice::Slice(cv::Mat data, cv::Vec3d origin, cv::Vec3d xvec, cv::Vec3d yvec)
    : sliceData_(data), origin_(origin), xvec_(xvec), yvec_(yvec)
{
}

cv::Mat Slice::draw() const
{
    auto debug = sliceData_.clone();

    // DEBUG
    debug.convertTo(debug, CV_8UC1, 1.0 / 255.0);
    cv::equalizeHist(debug, debug);

    debug.convertTo(debug, CV_8UC3);
    cv::cvtColor(debug, debug, cv::COLOR_GRAY2BGR);
    cv::Point imcenter(debug.cols / 2, debug.rows / 2);

    // Draw circle at pixel representing center
    cv::circle(debug, imcenter, 0, BGR_MAGENTA, -1);
    return debug;
}
