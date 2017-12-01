#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "vc/core/types/VolumePkg.hpp"

namespace py = pybind11;
namespace vc = volcart;

void init_VolumePkg(py::module&);

void init_VolumePkg(py::module& m)
{
    /** Class */
    py::class_<vc::VolumePkg, vc::VolumePkg::Pointer> c(m, "VolumePkg");
    c.doc() = "The interface to the VolumePkg (.volpkg) file format.";

    /** Constructors */
    c.def(
        py::init([](std::string path) { return vc::VolumePkg::New(path); }),
        py::arg("path"), "Load a VolumePkg from a filesystem path");

    /** Metadata */
    c.def("name", &vc::VolumePkg::name, "Human-readable VolumePkg name");
    c.def("version", &vc::VolumePkg::version, "The VolumePkg version number");
    c.def(
        "materialThickness", &vc::VolumePkg::materialThickness,
        "Approximate thickness of a material layer in microns");

    /** Volumes */
    c.def(
        "numberOfVolumes", &vc::VolumePkg::numberOfVolumes,
        "Number of Volumes in the package");
    c.def("volumeIDs", &vc::VolumePkg::volumeIDs, "Get the list of Volume IDs");
    c.def(
        "volumeNames", &vc::VolumePkg::volumeNames,
        "Get the list of human-readable Volume names");
    c.def(
         "volume", py::overload_cast<>(&vc::VolumePkg::volume),
         "Get the default Volume")
        .def(
            "volume",
            py::overload_cast<const vc::Volume::Identifier&>(
                &vc::VolumePkg::volume),
            py::arg("id"), "Get a Volume by ID");
}
