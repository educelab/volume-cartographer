#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/meshing/ScaleMesh.hpp"
#include "vc/testing/TestingUtils.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/FlatteningError.hpp"

using namespace volcart;
using namespace volcart::meshing;
using namespace volcart::shapes;
using namespace volcart::texturing;
using namespace volcart::testing;

TEST(FlatteningError, MetricsNoDistortion)
{
    // Construct arch
    Arch arch;
    auto mesh3D = arch.itkMesh();
    // Flatten
    AngleBasedFlattening abf;
    abf.setMesh(mesh3D);
    auto mesh2D = abf.compute();
    // Calculate metric
    auto metrics = LStretch(mesh3D, mesh2D);
    // Arch should flatten without distortion
    SmallOrClose(metrics.l2, 1.0);
    SmallOrClose(metrics.lInf, 1.0);
    using ::testing::DoubleNear;
    using ::testing::Each;
    EXPECT_THAT(metrics.faceL2, Each(DoubleNear(1.0, 1e-7)));
    EXPECT_THAT(metrics.faceLInf, Each(DoubleNear(1.0, 1e-7)));
}

TEST(FlatteningError, MetricsScale)
{
    using ::testing::DoubleNear;
    using ::testing::Each;

    // Construct plane
    Plane plane;
    auto mesh3D = plane.itkMesh();

    // Double distortion: 2D->3D
    auto factor = 0.5;
    auto expected = 1.0 / factor;
    auto mesh2D = ScaleMesh(mesh3D, factor);
    auto metrics = LStretch(mesh3D, mesh2D);
    SmallOrClose(metrics.l2, expected);
    SmallOrClose(metrics.lInf, expected);
    EXPECT_THAT(metrics.faceL2, Each(DoubleNear(expected, 1e-7)));
    EXPECT_THAT(metrics.faceLInf, Each(DoubleNear(expected, 1e-7)));

    // Half distortion: 2D->3D
    factor = 2.0;
    expected = 1.0 / factor;
    mesh2D = ScaleMesh(mesh3D, factor);
    metrics = LStretch(mesh3D, mesh2D);
    SmallOrClose(metrics.l2, expected);
    SmallOrClose(metrics.lInf, expected);
    EXPECT_THAT(metrics.faceL2, Each(DoubleNear(expected, 1e-7)));
    EXPECT_THAT(metrics.faceLInf, Each(DoubleNear(expected, 1e-7)));
}

auto main(int argc, char** argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    volcart::logging::SetLogLevel("off");
    return RUN_ALL_TESTS();
}