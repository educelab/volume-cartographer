#include <gtest/gtest.h>

#include <chrono>

#include "vc/core/shapes/Plane.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;
namespace vct = volcart::texturing;

class PPMGeneratorTest : public testing::TestWithParam<int>
{
};

TEST_P(PPMGeneratorTest, PerformanceTest)
{
    // Build Plane
    vc::shapes::Plane plane(GetParam(), GetParam());

    // Get ITK Mesh
    auto mesh = plane.itkMesh();

    // Generate UV map
    vc::UVMap uvMap;
    size_t pID = 0;
    for (int y = 0; y < GetParam(); y++) {
        auto v = double(y) / (GetParam() - 1);
        for (int x = 0; x < GetParam(); x++) {
            auto u = double(x) / (GetParam() - 1);
            uvMap.set(pID++, {u, v});
        }
    }

    // Setup PPM Generator
    vct::PPMGenerator ppmGenerator;
    ppmGenerator.setDimensions(100, 100);
    ppmGenerator.setMesh(mesh);
    ppmGenerator.setUVMap(uvMap);

    // Generate PPM
    auto start = std::chrono::system_clock::now();
    ppmGenerator.compute();
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> secs = end - start;
    std::cout << "size: " << GetParam() << " | elapsed: " << secs.count()
              << "s\n";
}

INSTANTIATE_TEST_SUITE_P(
    PerformanceTest,
    PPMGeneratorTest,
    testing::Values(2, 5, 10, 50, 100, 500, 1000));