#include <gtest/gtest.h>

#include "vc/core/shapes/Plane.hpp"
#include "vc/meshing/CalculateNormals.hpp"

class PlaneFixture : public ::testing::Test
{
public:
    PlaneFixture() { inMesh = Plane.itkMesh(); }

    volcart::shapes::Plane Plane;
    volcart::ITKMesh::Pointer inMesh, outMesh;
};

TEST_F(PlaneFixture, ComputePlaneNormalsTest)
{

    volcart::meshing::CalculateNormals calcNorm(inMesh);
    calcNorm.compute();
    outMesh = calcNorm.getMesh();

    for (auto p_it = outMesh->GetPoints()->Begin();
         p_it != outMesh->GetPoints()->End(); ++p_it) {
        volcart::ITKPixel outNormal, inNormal;
        outMesh->GetPointData(p_it.Index(), &outNormal);
        inMesh->GetPointData(p_it.Index(), &inNormal);

        EXPECT_DOUBLE_EQ(outNormal[0], inNormal[0]);
        EXPECT_DOUBLE_EQ(outNormal[1], inNormal[1]);
        EXPECT_DOUBLE_EQ(outNormal[2], inNormal[2]);
    }
}
