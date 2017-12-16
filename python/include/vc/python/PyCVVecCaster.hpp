#pragma once

#include <opencv2/core.hpp>
#include <pybind11/pybind11.h>

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
    PYBIND11_TYPE_CASTER(Vector, _("tuple"));

    /** From Tuple -> cv::Vec */
    bool load(handle src, bool)
    {
        // Make sure it's an array
        if (!isinstance<tuple>(src)) {
            return false;
        }

        // Cast the object to tuple
        auto args = reinterpret_borrow<tuple>(src);

        // Num. elements check
        if (args.size() != Dim) {
            pybind11::print("Incorrect size for vector:  ", args.size());
            return false;
        }

        // Assign the values
        value = Vector();
        for (int i = 0; i < Dim; i++) {
            value[i] = args[i].cast<T>();
        }

        return true;
    }

    /** From cv::Vec -> Tuple */
    static handle cast(Vector src, return_value_policy, handle)
    {
        // Construct a py::array directly
        tuple value(Dim);
        for (int i = 0; i < Dim; i++) {
            value[i] = src[i];
        }

        return value.release();
    }
};
}
}