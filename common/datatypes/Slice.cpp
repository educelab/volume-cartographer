#include <opencv2/opencv.hpp>

#include "Slice.h"

#define BGR_MAGENTA cv::Scalar(0xFF, 0, 0xFF)

Slice::Slice(cv::Mat data, cv::Vec3d origin, cv::Vec3d xvec, cv::Vec3d yvec)
    : sliceData_(data), origin_(origin), xvec_(xvec), yvec_(yvec)
{
}

cv::Vec3d Slice::sliceCoordToVoxelCoord(const cv::Point sliceCoords) const
{
    return origin_ + (sliceCoords.x * xvec_ + sliceCoords.y * yvec_);
}

cv::Mat Slice::draw() const
{
    auto debug = sliceData_.clone();
    debug /= 255.0;
    debug.convertTo(debug, CV_8UC3);
    cvtColor(debug, debug, CV_GRAY2BGR);
    cv::Point imcenter(debug.cols / 2, debug.rows / 2);

    // Draw circle at pixel representing center
    circle(debug, imcenter, 0, BGR_MAGENTA, -1);
    return debug;
}
