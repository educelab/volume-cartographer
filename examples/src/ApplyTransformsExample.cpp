#include "vc/core/io/MeshIO.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/types/Transforms.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/texturing/PPMGenerator.hpp"

using namespace volcart;
using namespace volcart::texturing;

auto main() -> int
{
    // Build transforms
    auto translate = AffineTransform::New();
    translate->translate(5, 5, 5);

    auto rotate = AffineTransform::New();
    rotate->rotate(90, 0, 0, 1);

    auto scale = AffineTransform::New();
    scale->scale(1, 2, 3);

    // ApplyTransform ITKMesh
    auto cube = shapes::Cube().itkMesh();
    WriteMesh("Cube_1_original.obj", cube);
    WriteMesh("Cube_2_translate.obj", ApplyTransform(cube, translate));
    WriteMesh("Cube_3_rotate.obj", ApplyTransform(cube, rotate));
    WriteMesh("Cube_4_scale.obj", ApplyTransform(cube, scale));

    // ApplyTransform PPM
    // Build Plane UVMap
    shapes::Plane plane(5, 5);
    auto mesh = plane.itkMesh();
    auto uvMap = UVMap::New();
    std::size_t id{0};
    for (const auto uv : range2D(5, 5)) {
        auto u = double(uv.first) / 4.0;
        auto v = double(uv.second) / 4.0;
        uvMap->set(id++, {u, v});
    }

    // Setup PPM Generator
    PPMGenerator ppmGenerator;
    ppmGenerator.setDimensions(100, 100);
    ppmGenerator.setMesh(mesh);
    ppmGenerator.setUVMap(uvMap);

    // Transform PPM
    auto ppm = ppmGenerator.compute();
    PerPixelMap::WritePPM("Plane_1_original.ppm", *ppm);
    PerPixelMap::WritePPM(
        "Plane_2_translate.ppm", ApplyTransform(*ppm, translate));
    PerPixelMap::WritePPM("Plane_3_rotate.ppm", ApplyTransform(*ppm, rotate));
    PerPixelMap::WritePPM("Plane_4_scale.ppm", ApplyTransform(*ppm, scale));

    // ApplyTransform PointSet
    using psio = PointSetIO<cv::Vec3d>;
    auto pts = plane.unorderedPoints();
    psio::WritePointSet("Plane_1_original_unordered.vcps", pts);
    psio::WritePointSet(
        "Plane_2_translate_unordered.vcps", ApplyTransform(pts, translate));
    psio::WritePointSet(
        "Plane_3_rotate_unordered.vcps", ApplyTransform(pts, rotate));
    psio::WritePointSet(
        "Plane_4_scale_unordered.vcps", ApplyTransform(pts, scale));

    // ApplyTransform OrderedPointSet
    auto opts = plane.orderedPoints();
    psio::WriteOrderedPointSet("Plane_1_original_ordered.vcps", opts);
    psio::WriteOrderedPointSet(
        "Plane_2_translate_ordered.vcps", ApplyTransform(opts, translate));
    psio::WriteOrderedPointSet(
        "Plane_3_rotate_ordered.vcps", ApplyTransform(opts, rotate));
    psio::WriteOrderedPointSet(
        "Plane_4_scale_ordered.vcps", ApplyTransform(opts, scale));
}