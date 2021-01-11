#include <pybind11/pybind11.h>

#include "vc/core/neighborhood/CuboidGenerator.hpp"
#include "vc/core/types/Volume.hpp"
#include "vc/python/PyCVMatCaster.hpp"
#include "vc/python/PyCVVecCaster.hpp"

namespace py = pybind11;
namespace vc = volcart;

void init_Volume(py::module& m);

void init_Volume(py::module& m)
{
    /** Class */
    py::class_<vc::Volume, vc::Volume::Pointer> c(m, "Volume");
    c.doc() =
        "Provides access to a volumetric dataset, such as a CT scan. "
        "Since these datasets are often very large, this class is "
        "streaming in nature: the volume slices are stored on disk and "
        "cached in memory when needed.";

    /** Constructors */
    // Load from path w/ metadata
    c.def(
        py::init([](std::string p) { return vc::Volume::New(p); }),
        py::arg("path"),
        "Load a Volume from a filesystem path. Requires "
        "a \'meta.json\' file describing the Volume in the provided path");

    /** Metadata */
    c.def("id", &vc::Volume::id, "The Volume ID");
    c.def("name", &vc::Volume::name, "The human-readable Volume name");
    c.def("width", &vc::Volume::sliceWidth, "Width of a slice image");
    c.def("height", &vc::Volume::sliceHeight, "Height of a slice image");
    c.def("slices", &vc::Volume::numSlices, "Number of slices");
    c.def("voxelSize", &vc::Volume::voxelSize, "Voxel size in microns");

    /** Caching policy */
    c.def(
        "setCacheCapacity", &vc::Volume::setCacheCapacity, py::arg("size"),
        "Set the maximum number of cached slices");
    c.def(
        "getCacheCapacity", &vc::Volume::getCacheCapacity,
        "Get the maximum number of cached slices");
    c.def(
        "getCacheSize", &vc::Volume::getCacheSize,
        "Get the current number of cached slices");
    c.def(
        "setCacheMemory", &vc::Volume::setCacheMemoryInBytes, py::arg("bytes"),
        "Set the maximum cache size in bytes");

    /** Slice Data */
    c.def(
        "slice", &vc::Volume::getSliceData, py::arg("z"),
        "Get a slice image by index");

    /** Voxel Data */
    c.def(
        "intensityAt",
        py::overload_cast<const cv::Vec3d&>(
            &vc::Volume::intensityAt, py::const_),
        "Get the intensity at a voxel position", py::arg_v("pos", "(x, y, z)"));
    c.def(
        "interpolateAt",
        py::overload_cast<const cv::Vec3d&>(
            &vc::Volume::interpolateAt, py::const_),
        "Get the interpolated intensity at a subvoxel position",
        py::arg_v("pos", "(x, y, z)"));

    /** Reslices and Subvolumes */
    c.def(
        "reslice", &vc::Volume::reslice,
        // clang-format off
        py::arg_v("center", "(x, y, z)"),
        py::arg_v("x_vec", cv::Vec3d{1, 0, 0}, "(1, 0, 0)"),
        py::arg_v("y_vec", cv::Vec3d{0, 1, 0}, "(0, 1, 0)"),
        py::arg("width") = 64,
        py::arg("height") = 64,
        "Generate an arbitrarily-oriented reslice image");
    // clang-format on

    c.def(
        "subvolume",
        [](vc::Volume& v, cv::Vec3d center, int rx, int ry, int rz,
           cv::Vec3d xvec, cv::Vec3d yvec, cv::Vec3d zvec) {
            vc::CuboidGenerator subvolume;
            subvolume.setSamplingRadius(rx, ry, rz);
            auto s = subvolume.compute(
                v.shared_from_this(), center, {xvec, yvec, zvec});
            auto buf = s.as_vector();
            size_t size = sizeof(uint16_t);
            std::string format = py::format_descriptor<uint16_t>::format();
            auto extents = s.extents();
            std::vector<size_t> strides{
                size * extents[1] * extents[0], size * extents[0], size};
            return py::array(py::buffer_info{
                                 buf.data(), size, format, 3, extents, strides})
                .release();
        },
        // clang-format off
        py::arg_v("center", "(x, y, z)"),
        py::arg("x_rad"),
        py::arg("y_rad"),
        py::arg("z_rad"),
        py::arg_v("x_vec", cv::Vec3d{1, 0, 0}, "(1, 0, 0)"),
        py::arg_v("y_vec", cv::Vec3d{0, 1, 0}, "(0, 1, 0)"),
        py::arg_v("z_vec", cv::Vec3d{0, 0, 1}, "(0, 0, 1)"),
        "Generate an arbitrarily-oriented subvolume");
    // clang-format on
}
