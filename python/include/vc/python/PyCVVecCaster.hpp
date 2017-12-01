#pragma once

#include <opencv2/core.hpp>
#include <pybind11/numpy.h>

namespace pybind11
{
namespace detail
{

/**
 * Numpy array <- cv::Vec3d -> Numpy array Caster
 */
template <>
struct type_caster<cv::Vec3d> {
public:
    PYBIND11_TYPE_CASTER(cv::Vec3d, _("numpy.array"));

    /** From np.array -> cv::Vec3d */
    bool load(handle src, bool)
    {
        // Make sure it's an array
        if (!isinstance<array>(src)) {
            return false;
        }

        // Cast the array to a double
        auto arr =
            array_t<double, array::c_style | array::forcecast>::ensure(src);
        if (!arr)
            return false;

        // Dimensionality check
        if (arr.ndim() != 1) {
            pybind11::print("Incorrect dims for Vec3d: ", arr.ndim());
            return false;
        }

        // Num. elements check
        if (arr.size() != 3) {
            pybind11::print("Incorrect size for Vec3d:  ", arr.size());
            return false;
        }

        // Assign the value
        value = cv::Vec3d(arr.data());

        return true;
    }

    /** From cv::Vec3d -> np.array */
    static handle cast(cv::Vec3d src, return_value_policy, handle)
    {
        // Construct a py::array directly
        return array(buffer_info{src.val,
                                 sizeof(double),
                                 format_descriptor<double>::format(),
                                 1,
                                 {3},
                                 {sizeof(double)}})
            .release();
    }
};
}
}