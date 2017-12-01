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
    py::class_<vc::Reslice> c(m, "Reslice");
    c.doc() = "Arbitrarily-oriented reslice of a Volume";

    /** Coordinate Transformation */
    c.def(
        "pixelToVoxelCoord",
        [](const vc::Reslice& r, double x, double y) -> cv::Vec3d {
            return r.sliceToVoxelCoord<double>({x, y});
        },
        py::arg("x"), py::arg("y"),
        "Convert a 2D Reslice coordinate into its 3D Volume coordinate");

    /** Image */
    c.def("data", &vc::Reslice::sliceData, "Get the Reslice image data");
}
