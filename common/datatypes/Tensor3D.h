#pragma once

#ifndef _VOLCART_TENSOR3D_H_
#define _VOLCART_TENSOR3D_H_

#include <vector>
#include <opencv2/core/mat.hpp>

namespace volcart
{

template <typename DType>
class Tensor3D
{
private:
    int32_t dx_;
    int32_t dy_;
    int32_t dz_;
    cv::Mat_ tensor_;

public:
    Tensor3D<DType>() = default;

    Tensor3D<DType>(const int32_t dx, const int32_t dy, const int32_t dz, const bool zero=true) :
        dx_(dx), dy_(dy), dz_(dz)
    {
        // XXX check if these dimensions are too large?
        constexpr int size[] = { dy_, dx_, dz_ };
        if (zero) {
            tensor_ = cv::Mat_<DType>(3, size, cv::Scalar(0));
        } else {
            tensor_ = cv::Mat_<DType>(3, size);
        }
    }

    const cv::Mat& slice(const int32_t z) const
    {
        return tensor_[z];
    }

    cv::Mat& slice(const int32_t z)
    {
        return tensor_[z];
    }

    const DType& operator()(const int32_t x, const int32_t y, const int32_t z) const
    {
        return tensor_(y, x, z);
    }

    DType& operator()(const int32_t x, const int32_t y, const int32_t z)
    {
        return tensor_(y, x, z);
    }

    const cv::Mat_& operator()(const Range& rx, const Range& ry, const Range& rz) const
    {
        const Range ranges[] = { ry, rx, rz };
        return tensor_(ranges);
    }
};
}

#endif
