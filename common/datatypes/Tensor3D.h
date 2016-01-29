#pragma once

#ifndef _VOLCART_TENSOR3D_H_
#define _VOLCART_TENSOR3D_H_

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

namespace volcart
{
template <typename DType>
class Tensor3D
{
private:
    int32_t dx_;
    int32_t dy_;
    int32_t dz_;
    std::vector<cv::Mat_<DType>> tensor_;

public:
    Tensor3D<DType>() = default;

    Tensor3D<DType>(const int32_t dx, const int32_t dy, const int32_t dz,
                    const bool zero = true)
        : dx_(dx), dy_(dy), dz_(dz)
    {
        tensor_.reserve(dz);

        // XXX check if these dimensions are too large?
        if (zero) {
            // Static vars are automatically initialized to zero, so it's a
            // convenient way to zero-initialize the tensor
            static DType d;
            for (int32_t i = 0; i < dz; ++i) {
                tensor_.emplace_back(dy, dx, d);
            }
        } else {
            for (int32_t i = 0; i < dz; ++i) {
                tensor_.emplace_back(dy, dx);
            }
        }
    }

    const cv::Mat_<DType>& xySlice(const int32_t z) const { return tensor_[z]; }
    cv::Mat_<DType>& xySlice(const int32_t z) { return tensor_[z]; }
    cv::Mat_<DType> xzSlice(const int32_t layer) const
    {
        cv::Mat_<DType> zSlice(dz_, dx_);
        for (int32_t z = 0; z < dz_; ++z) {
            tensor_[z].row(layer).copyTo(zSlice.row(z));
        }
        return zSlice;
    }

    const DType& operator()(const int32_t x, const int32_t y,
                            const int32_t z) const
    {
        assert(x < dx_ && x >= 0 && "index out of range\n");
        assert(y < dy_ && y >= 0 && "index out of range\n");
        assert(z < dz_ && z >= 0 && "index out of range\n");
        return tensor_[z](y, x);
    }

    DType& operator()(const int32_t x, const int32_t y, const int32_t z)
    {
        assert(x < dx_ && x >= 0 && "index out of range\n");
        assert(y < dy_ && y >= 0 && "index out of range\n");
        assert(z < dz_ && z >= 0 && "index out of range\n");
        return tensor_[z](y, x);
    }

    std::unique_ptr<DType[]> buffer(void) const
    {
        auto buf = std::unique_ptr<DType[]>(new DType[dx_ * dy_ * dz_]);
        for (int32_t z = 0; z < dz_; ++z) {
            for (int32_t y = 0; y < dy_; ++y) {
                for (int32_t x = 0; x < dx_; ++x) {
                    buf[z * dx_ * dy_ + y * dx_ + x] = tensor_[z](y, x);
                }
            }
        }
        return buf;
    }

    int32_t dx() const { return dx_; }
    int32_t dy() const { return dy_; }
    int32_t dz() const { return dz_; }
};
}

#endif
