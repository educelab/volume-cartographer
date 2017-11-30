#include <pybind11/pybind11.h>

#include "vc/core/types/Volume.hpp"
#include "vc/python/PyMatCaster.hpp"

namespace py = pybind11;
namespace vc = volcart;

void init_Volume(py::module&);

void init_Volume(py::module& m)
{
    /** Class */
    py::class_<vc::Volume> vol(m, "Volume");

    /** Constructors */
    // Load from path w/ metadata
    vol.def(py::init([](std::string p) { return vc::Volume(p); }));

    /** Metadata */
    vol.def("id", &vc::Volume::id);
    vol.def("name", &vc::Volume::name);
    vol.def("sliceWidth", &vc::Volume::sliceWidth);
    vol.def("sliceHeight", &vc::Volume::sliceHeight);
    vol.def("numSlices", &vc::Volume::numSlices);
    vol.def("voxelSize", &vc::Volume::voxelSize);

    /** Slice Data */
    vol.def("getSlice", &vc::Volume::getSliceData);
}