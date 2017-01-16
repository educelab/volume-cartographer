#pragma once

#include <cassert>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>

namespace volcart
{

template <typename DType>
class Tensor3D
{
private:
    std::vector<cv::Mat_<DType>> tensor_;

public:
    size_t dx;
    size_t dy;
    size_t dz;

    Tensor3D<DType>() = default;

    Tensor3D<DType>(size_t x, size_t y, size_t z, bool zero = true)
        : dx(x), dy(y), dz(z)
    {
        tensor_.reserve(dz);

        // XXX check if these dimensions are too large?
        if (zero) {
            // Static vars are automatically initialized to zero, so it's a
            // convenient way to zero-initialize the tensor
            static DType d;
            for (size_t i = 0; i < dz; ++i) {
                tensor_.emplace_back(dy, dx, d);
            }
        } else {
            for (size_t i = 0; i < dz; ++i) {
                tensor_.emplace_back(dy, dx);
            }
        }
    }

    const cv::Mat_<DType>& xySlice(size_t z) const { return tensor_[z]; }
    cv::Mat_<DType>& xySlice(size_t z) { return tensor_[z]; }
    cv::Mat_<DType> xzSlice(size_t layer) const
    {
        cv::Mat_<DType> zSlice(dz, dx);
        for (size_t z = 0; z < dz; ++z) {
            tensor_[z].row(layer).copyTo(zSlice.row(z));
        }
        return zSlice;
    }

    const DType& operator()(size_t x, size_t y, size_t z) const
    {
        assert(x < dx && x >= 0 && "index out of range");
        assert(y < dy && y >= 0 && "index out of range");
        assert(z < dz && z >= 0 && "index out of range");
        return tensor_[z](y, x);
    }

    DType& operator()(size_t x, size_t y, size_t z)
    {
        assert(x < dx && x >= 0 && "index out of range");
        assert(y < dy && y >= 0 && "index out of range");
        assert(z < dz && z >= 0 && "index out of range");
        return tensor_[z](y, x);
    }

    std::unique_ptr<DType[]> buffer() const
    {
        auto buf = std::unique_ptr<DType[]>(new DType[dx * dy * dz]);
        for (size_t z = 0; z < dz; ++z) {
            for (size_t y = 0; y < dy; ++y) {
                for (size_t x = 0; x < dx; ++x) {
                    buf[z * dx * dy + y * dx + x] = tensor_[z](y, x);
                }
            }
        }
        return buf;
    }
};
}

template <typename DType>
std::ostream& operator<<(
    std::ostream& s, const volcart::Tensor3D<DType>& tensor)
{
    for (size_t z = 0; z < tensor.dz; ++z) {
        s << tensor.xySlice(z) << std::endl;
    }
    return s;
}
