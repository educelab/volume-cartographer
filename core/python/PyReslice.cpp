#include <pybind11/pybind11.h>

#include "vc/core/types/Reslice.hpp"
#include "vc/python/PyCVMatCaster.hpp"
#include "vc/python/PyCVVecCaster.hpp"

namespace py = pybind11;
namespace vc = volcart;

void init_Reslice(py::module&);

void init_Reslice(py::module& m)
{
    /** Class */
    py::class_<vc::Reslice> res(m, "Reslice");

    /** Image */
    res.def("data", &vc::Reslice::sliceData);
}
