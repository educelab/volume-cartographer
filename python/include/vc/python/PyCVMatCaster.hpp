#pragma once

#include <opencv2/core.hpp>
#include <pybind11/numpy.h>

/**
 * cv::Mat -> Numpy array caster
 * Inspired by: https://github.com/pybind/pybind11/issues/538
 */
namespace pybind11
{
namespace detail
{

template <>
struct type_caster<cv::Mat> {
public:
    PYBIND11_TYPE_CASTER(cv::Mat, _("array"));

    static handle cast(cv::Mat src, return_value_policy, handle)
    {
        size_t size;
        std::string format;
        auto dims = src.dims;

        switch (src.type()) {
            case CV_8U:
                size = sizeof(uint8_t);
                format = format_descriptor<uint8_t>::format();
                break;
            case CV_8S:
                size = sizeof(int8_t);
                format = format_descriptor<int8_t>::format();
                break;
            case CV_16U:
                size = sizeof(uint16_t);
                format = format_descriptor<uint16_t>::format();
                break;
            case CV_16S:
                size = sizeof(int16_t);
                format = format_descriptor<int16_t>::format();
                break;
            case CV_32F:
                size = sizeof(float);
                format = format_descriptor<float>::format();
                break;
            default:
                throw std::runtime_error("unsupported image type");
        }

        std::vector<size_t> extents;
        std::vector<size_t> strides;

        if (dims == 2) {
            extents = {src.rows, src.cols};
            strides = {size * src.cols, size};
        } else if (dims == 3) {
            extents = {src.rows, src.cols, src.channels()};
            strides = {size * src.cols * src.channels(), size * src.channels(),
                       size};
        } else {
            throw std::runtime_error("unsupported number of dims");
        }

        return array(
                   buffer_info{src.data, size, format, dims, extents, strides})
            .release();
    }
};
}
}
