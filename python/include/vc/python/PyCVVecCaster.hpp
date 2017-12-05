#pragma once

#include <opencv2/core.hpp>
#include <pybind11/numpy.h>

namespace pybind11
{
namespace detail
{

/**
 * Numpy array <- cv::Vec -> Numpy array Caster
 */
template <typename T, int Dim>
struct type_caster<cv::Vec<T, Dim>> {
public:
    using Vector = cv::Vec<T, Dim>;
    PYBIND11_TYPE_CASTER(Vector, _("numpy.array"));

    /** From np.array -> cv::Vec */
    bool load(handle src, bool)
    {
        // Make sure it's an array
        if (!isinstance<array>(src)) {
            return false;
        }

        // Cast the array to a double
        auto arr = array_t<T, array::c_style | array::forcecast>::ensure(src);
        if (!arr)
            return false;

        // Dimensionality check
        if (arr.ndim() != 1) {
            pybind11::print("Incorrect dims for vector: ", arr.ndim());
            return false;
        }

        // Num. elements check
        if (arr.size() != Dim) {
            pybind11::print("Incorrect size for vector:  ", arr.size());
            return false;
        }

        // Assign the value
        value = Vector(arr.data());

        return true;
    }

    /** From cv::Vec -> np.array */
    static handle cast(Vector src, return_value_policy, handle)
    {
        // Construct a py::array directly
        return array(buffer_info{src.val,
                                 sizeof(T),
                                 format_descriptor<T>::format(),
                                 1,
                                 {Dim},
                                 {sizeof(T)}})
            .release();
    }
};
}
}