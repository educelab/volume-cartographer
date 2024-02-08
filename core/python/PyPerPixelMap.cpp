#include <cstddef>
#include <tuple>

#include <pybind11/pybind11.h>

#include "vc/core/types/PerPixelMap.hpp"
#include "vc/python/PyCVVecCaster.hpp"

namespace py = pybind11;
namespace vc = volcart;

void init_PerPixelMap(py::module&);

void init_PerPixelMap(py::module& m)
{
    /** Class */
    py::class_<vc::PerPixelMap> c(m, "PerPixelMap");
    c.doc() =
        "A raster of a UVMap that provides a per-pixel mapping between "
        "a Volume and a Texture generated from that volume";

    /** Metadata */
    c.def("width", &vc::PerPixelMap::width);
    c.def("height", &vc::PerPixelMap::height);

    /** Element Access */
    c.def(
        "__getitem__",
        [](vc::PerPixelMap& p, std::tuple<std::size_t, std::size_t> pos) {
            return p(std::get<0>(pos), std::get<1>(pos));
        },
        py::arg("pos[y, x]"), "Get the mapping for a pixel by coordinate");
    c.def(
        "get",
        py::overload_cast<std::size_t, std::size_t>(
            &vc::PerPixelMap::operator()),
        py::arg("y"), py::arg("x"),
        "Get the mapping for a pixel by coordinate");
    c.def(
        "hasMapping", &vc::PerPixelMap::hasMapping, py::arg("y"), py::arg("x"),
        "Return whether a pixel has a mapping");

    /** IO */
    // Note: Defined in the module, not the class
    m.def(
        "ReadPPM",
        [](std::string path) -> vc::PerPixelMap {
            return vc::PerPixelMap::ReadPPM(path);
        },
        py::arg("path"), "Load a PerPixelMap from a ppm file path");
}