#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "vc/core/types/VolumePkg.hpp"

namespace py = pybind11;
namespace vc = volcart;

void init_VolumePkg(py::module&);

void init_VolumePkg(py::module& m)
{
    /** Class */
    py::class_<vc::VolumePkg, vc::VolumePkg::Pointer> v(m, "VolumePkg");

    /** Constructors */
    v.def(py::init([](std::string p) { return vc::VolumePkg::New(p); }));

    /** Metadata */
    v.def("name", &vc::VolumePkg::name);
    v.def("version", &vc::VolumePkg::version);
    v.def("materialThickness", &vc::VolumePkg::materialThickness);

    /** Volumes */
    v.def("numberOfVolumes", &vc::VolumePkg::numberOfVolumes);
    v.def("volumeIDs", &vc::VolumePkg::volumeIDs);
    v.def("volumeNames", &vc::VolumePkg::volumeNames);
    v.def(
        "volume",
        [](vc::VolumePkg& vpkg, vc::Volume::Identifier id) {
            if (id.empty()) {
                return vpkg.volume();
            } else {
                return vpkg.volume(id);
            }
        },
        py::arg("id") = "");
}
