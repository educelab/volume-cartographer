#include <pybind11/pybind11.h>

#include "vc/core/types/Volume.hpp"
#include "vc/python/PyCVMatCaster.hpp"
#include "vc/python/PyCVVecCaster.hpp"

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

    /** Caching policy */
    vol.def("setCacheCapacity", &vc::Volume::setCacheCapacity, py::arg("size"));
    vol.def("getCacheCapacity", &vc::Volume::getCacheCapacity);
    vol.def("getCacheSize", &vc::Volume::getCacheSize);
    vol.def(
        "setCacheMemory", &vc::Volume::setCacheMemoryInBytes, py::arg("bytes"));

    /** Slice Data */
    vol.def("getSlice", &vc::Volume::getSliceData, py::arg("idx"));

    /** Voxel Data */
    vol.def(
        "intensityAt",
        py::overload_cast<int, int, int>(&vc::Volume::intensityAt, py::const_),
        "Get the intensity value at a voxel position", py::arg("x"),
        py::arg("y"), py::arg("z"));
    vol.def(
        "interpIntensityAt",
        py::overload_cast<double, double, double>(
            &vc::Volume::interpolatedIntensityAt, py::const_),
        "Get the intensity value at a subvoxel position", py::arg("x"),
        py::arg("y"), py::arg("z"));

    /** Reslices and Subvolumes */
    vol.def(
        "reslice", &vc::Volume::reslice,
        // clang-format off
        py::arg("center"),
        py::arg_v("x_vec", cv::Vec3d{1, 0, 0}, "{1,0,0}"),
        py::arg_v("y_vec", cv::Vec3d{0, 1, 0}, "{0,1,0}"),
        py::arg_v("width", int(64), "64"),
        py::arg_v("height", int(64), "64"));
    // clang-format on

    vol.def(
        "subvolume",
        [](vc::Volume& v, cv::Vec3d c, int rx, int ry, int rz, cv::Vec3d xvec,
           cv::Vec3d yvec, cv::Vec3d zvec) {
            auto s = v.getVoxelNeighborsInterpolated<uint16_t>(
                c, rx, ry, rz, xvec, yvec, zvec);
            auto buf = s.buffer();
            size_t size = sizeof(uint16_t);
            std::string format = py::format_descriptor<uint16_t>::format();
            std::vector<size_t> extents{s.dz(), s.dy(), s.dx()};
            std::vector<size_t> strides{size * s.dy() * s.dx(), size * s.dy(),
                                        size};
            return py::array(py::buffer_info{buf.release(), size, format, 3,
                                             extents, strides})
                .release();
        },
        // clang-format off
        py::arg("center"),
        py::arg("x_rad"),
        py::arg("y_rad"),
        py::arg("z_rad"),
        py::arg_v("x_vec", cv::Vec3d{1, 0, 0}, "{1,0,0}"),
        py::arg_v("y_vec", cv::Vec3d{0, 1, 0}, "{0,1,0}"),
        py::arg_v("z_vec", cv::Vec3d{0, 0, 1}, "{0,0,1}"));
    // clang-format on
}