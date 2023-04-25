#include <gtest/gtest.h>

#include <chrono>

#include "vc/core/shapes/Plane.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;
namespace vct = volcart::texturing;

class PPMGeneratorTest : public testing::TestWithParam<int>
{
};

TEST(PPMGeneratorTest, RegressionTest)
{
    // Build Plane UVMap
    vc::shapes::Plane plane(5, 5);
    auto mesh = plane.itkMesh();
    auto uvMap = vc::UVMap::New();
    std::size_t id{0};
    for (const auto uv : vc::range2D(5, 5)) {
        auto u = double(uv.first) / 4.0;
        auto v = double(uv.second) / 4.0;
        uvMap->set(id++, {u, v});
    }

    // Setup PPM Generator
    vct::PPMGenerator ppmGenerator;
    ppmGenerator.setDimensions(100, 100);
    ppmGenerator.setMesh(mesh);
    ppmGenerator.setUVMap(uvMap);

    // Generate PPM
    auto ppm = ppmGenerator.compute();

    // Compare against existing PPM
    auto expected = vc::PerPixelMap::ReadPPM("PPMGenerator_100x100.ppm");

    // Compare mappings
    for (const auto [y, x] : vc::range2D(100, 100)) {
        EXPECT_EQ(ppm->hasMapping(y, x), expected.hasMapping(y, x));

        if (not ppm->hasMapping(y, x)) {
            continue;
        }

        EXPECT_EQ(ppm->getMapping(y, x), expected(y, x));
        EXPECT_EQ(
            ppm->cellMap().at<int32_t>(y, x),
            expected.cellMap().at<int32_t>(y, x));
    }
}

TEST_P(PPMGeneratorTest, PerformanceTest)
{
    // Build Plane
    vc::shapes::Plane plane(GetParam(), GetParam());

    // Get ITK Mesh
    auto mesh = plane.itkMesh();

    // Generate UV map
    auto uvMap = vc::UVMap::New();
    size_t pID = 0;
    for (int y = 0; y < GetParam(); y++) {
        auto v = double(y) / (GetParam() - 1);
        for (int x = 0; x < GetParam(); x++) {
            auto u = double(x) / (GetParam() - 1);
            uvMap->set(pID++, {u, v});
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
