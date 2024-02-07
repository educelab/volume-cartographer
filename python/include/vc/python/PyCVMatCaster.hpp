#pragma once

/** @file */

#include <cstddef>
#include <cstdint>

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
        std::size_t size;
        std::string format;
        auto dims = src.dims;

        switch (src.type()) {
            case CV_8U:
                size = sizeof(std::uint8_t);
                format = format_descriptor<std::uint8_t>::format();
                break;
            case CV_8S:
                size = sizeof(std::int8_t);
                format = format_descriptor<std::int8_t>::format();
                break;
            case CV_16U:
                size = sizeof(std::uint16_t);
                format = format_descriptor<std::uint16_t>::format();
                break;
            case CV_16S:
                size = sizeof(std::int16_t);
                format = format_descriptor<std::int16_t>::format();
                break;
            case CV_32F:
                size = sizeof(float);
                format = format_descriptor<float>::format();
                break;
            default:
                throw std::runtime_error("unsupported image type");
        }

        std::vector<std::size_t> extents;
        std::vector<std::size_t> strides;

        if (dims == 2) {
            extents = {
                static_cast<unsigned long>(src.rows),
                static_cast<unsigned long>(src.cols)};
            strides = {size * src.cols, size};
        } else if (dims == 3) {
            extents = {
                static_cast<unsigned long>(src.rows),
                static_cast<unsigned long>(src.cols),
                static_cast<unsigned long>(src.channels())};
            strides = {
                size * src.cols * src.channels(), size * src.channels(), size};
        } else {
            throw std::runtime_error("unsupported number of dims");
        }

        return array(buffer_info{
                         src.data, static_cast<std::ssize_t>(size), format,
                         dims, extents, strides})
            .release();
    }
};
}  // namespace detail
}  // namespace pybind11
