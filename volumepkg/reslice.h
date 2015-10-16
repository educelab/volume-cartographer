#pragma once

#ifndef VC_SLICE_H
#define VC_SLICE_H

#include <opencv2/core/core.hpp>

// A simple wrapper around a cv::Mat of data sliced from the volume at an arbitrary x and y vector and an origin point.
class Reslice {
public:
    Reslice(cv::Mat, cv::Vec3f, cv::Vec3f, cv::Vec3f);

    cv::Vec3f sliceCoordToVoxelCoord(const cv::Point) const;

    const cv::Mat sliceData() const { return sliceData_; }

    void draw() const;

private:
    cv::Mat sliceData_;
    cv::Vec3f origin_;
    cv::Vec3f xvec_;
    cv::Vec3f yvec_;
};

#endif //VC_SLICE_H
