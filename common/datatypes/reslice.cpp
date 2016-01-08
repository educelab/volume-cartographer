#include <opencv2/opencv.hpp>

#include "reslice.h"


#define BGR_MAGENTA cv::Scalar(0xFF, 0,    0xFF)


Reslice::Reslice(cv::Mat data, cv::Vec3d origin, cv::Vec3d xvec, cv::Vec3d yvec) :
        sliceData_(data), origin_(origin), xvec_(xvec), yvec_(yvec) {
}

cv::Vec3d Reslice::sliceCoordToVoxelCoord(const cv::Point sliceCoords) const {
    return origin_ + (sliceCoords.x * xvec_ + sliceCoords.y * yvec_);
}

void Reslice::draw() const {
    auto debug = sliceData_.clone();
    debug /= 255.0;
    debug.convertTo(debug, CV_8UC3);
    cvtColor(debug, debug, CV_GRAY2BGR);
    cv::Point imcenter(debug.cols / 2, debug.rows / 2);

    // Draw circle at pixel representing center
    circle(debug, imcenter, 0, BGR_MAGENTA, -1);
    namedWindow("Reslice", cv::WINDOW_NORMAL);
    imshow("Reslice", debug);
}
