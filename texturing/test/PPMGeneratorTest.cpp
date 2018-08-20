#define BOOST_TEST_MODULE PPMGenerator

#include <chrono>

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "vc/core/shapes/Plane.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;
namespace vct = volcart::texturing;

namespace data = boost::unit_test::data;

// Test performance relative to mesh size
BOOST_DATA_TEST_CASE(Performance, data::make({2, 5, 10, 50, 100, 500, 1000}))
{
    // Build Plane
    vc::shapes::Plane plane(sample, sample);

    // Get ITK Mesh
    auto mesh = plane.itkMesh();

    // Generate UV map
    vc::UVMap uvMap;
    size_t pID = 0;
    for (int y = 0; y < sample; y++) {
        auto v = double(y) / (sample - 1);
        for (int x = 0; x < sample; x++) {
            auto u = double(x) / (sample - 1);
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
    std::cout << "size: " << sample << " | elapsed: " << secs.count() << "s\n";
}
