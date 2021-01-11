#pragma once

/**
 * @file
 *
 * @ingroup Math
 */

#include <cassert>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>

namespace volcart
{
/**
 * @class Tensor3D
 * @brief A 3rd-order tensor object
 *
 * Used by Volume to store gradients and structure tensors.
 *
 * @ingroup Types
 */
template <typename DType>
class Tensor3D
{
public:
    /**@{*/
    /** @brief Default constructor */
    Tensor3D<DType>() = default;

    /** @brief Creates a Tensor3D, sets the dimensions and whether it's full of
     * zeroes.
     * @param x The size of the X-axis
     * @param y The size of the Y-axis
     * @param z The size of the Z-axis
     * @param zero If true, initialize the Tensor with zeroes
     */
    Tensor3D<DType>(size_t x, size_t y, size_t z, bool zero = true)
        : dx_(x), dy_(y), dz_(z)
    {
        tensor_.reserve(dz_);

        // XXX check if these dimensions are too large?
        if (zero) {
            // Static vars are automatically initialized to zero, so it's a
            // convenient way to zero-initialize the tensor
            static DType d;
            for (size_t i = 0; i < dz_; ++i) {
                tensor_.emplace_back(dy_, dx_, d);
            }
        } else {
            for (size_t i = 0; i < dz_; ++i) {
                tensor_.emplace_back(dy_, dx_);
            }
        }
    }
    /**@}*/

    /**@{*/
    /** @brief Get the tensor value at x, y, z
     */
    const DType& operator()(size_t x, size_t y, size_t z) const
    {
        assert(x < dx_ && x >= 0 && "index out of range");
        assert(y < dy_ && y >= 0 && "index out of range");
        assert(z < dz_ && z >= 0 && "index out of range");
        return tensor_[z](y, x);
    }

    /** @copydoc operator()() */
    DType& operator()(size_t x, size_t y, size_t z)
    {
        assert(x < dx_ && x >= 0 && "index out of range");
        assert(y < dy_ && y >= 0 && "index out of range");
        assert(z < dz_ && z >= 0 && "index out of range");
        return tensor_[z](y, x);
    }
    /**@}*/

    /**@{*/
    /** @brief Get the size of the X-axis */
    size_t dx() { return dx_; }

    /** @copydoc dx() */
    size_t dx() const { return dx_; }

    /** @brief Get the size of the Y-axis */
    size_t dy() { return dy_; }

    /** @copydoc dy() */
    size_t dy() const { return dy_; }

    /** @brief Get the size of the Z-axis */
    size_t dz() { return dz_; }

    /** @copydoc dz() */
    size_t dz() const { return dz_; }
    /**@}*/

    /**@{*/
    /** @brief Get an XY cross section of the Tensor at z */
    const cv::Mat_<DType>& xySlice(size_t z) const { return tensor_[z]; }

    /** @copydoc xySlice() */
    cv::Mat_<DType>& xySlice(size_t z) { return tensor_[z]; }

    /** @brief Get an XZ cross section of the Tensor at y */
    cv::Mat_<DType> xzSlice(size_t y) const
    {
        cv::Mat_<DType> zSlice(dz_, dx_);
        for (size_t z = 0; z < dz_; ++z) {
            tensor_[z].row(y).copyTo(zSlice.row(z));
        }
        return zSlice;
    }
    /**@}*/

    /**@{*/
    /** @brief Get a copy of the Tensor as a raw array */
    std::unique_ptr<DType[]> buffer() const
    {
        auto buf = std::make_unique<DType[]>(dx_ * dy_ * dz_);
        for (size_t z = 0; z < dz_; ++z) {
            for (size_t y = 0; y < dy_; ++y) {
                for (size_t x = 0; x < dx_; ++x) {
                    buf[z * dx_ * dy_ + y * dx_ + x] = tensor_[z](y, x);
                }
            }
        }
        return buf;
    }
    /**@}*/

private:
    /** Tensor storage */
    std::vector<cv::Mat_<DType>> tensor_;
    /** Size of the X-axis */
    size_t dx_;
    /** Size of the Y-axis */
    size_t dy_;
    /** Size of the Z-axis */
    size_t dz_;
};
}  // namespace volcart

/**
 * @brief Write a Tensor3D to an output stream
 * @memberof volcart::Tensor3D
 */
template <typename DType>
std::ostream& operator<<(
    std::ostream& s, const volcart::Tensor3D<DType>& tensor)
{
    for (size_t z = 0; z < tensor.dz_; ++z) {
        s << tensor.xySlice(z) << std::endl;
    }
    return s;
}
