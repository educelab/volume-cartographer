#pragma once

#include <opencv2/core.hpp>

// A simple wrapper around a cv::Mat of data sliced from the volume at an
// arbitrary x and y vector and an origin point.
class Slice
{
public:
    Slice(cv::Mat data, cv::Vec3d origin, cv::Vec3d xvec, cv::Vec3d yvec)
        : sliceData_{std::move(data)}, origin_{origin}, xvec_{xvec}, yvec_{yvec}
    {
    }

    template <typename T>
    cv::Vec3d sliceToVoxelCoord(const cv::Point_<T>& resliceCoord) const
    {
        return origin_ + (resliceCoord.x * xvec_ + resliceCoord.y * yvec_);
    }

    const cv::Mat& sliceData() const { return sliceData_; }
    cv::Mat draw() const;

private:
    cv::Mat sliceData_;
    cv::Vec3d origin_;
    cv::Vec3d xvec_;
    cv::Vec3d yvec_;
};