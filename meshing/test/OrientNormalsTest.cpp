#include <gtest/gtest.h>

#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/meshing/OrientNormals.hpp"

using namespace volcart;
using namespace volcart::shapes;
using namespace volcart::meshing;

inline void ExpectNormalEq(
    const ITKMesh::Pointer& result, const ITKMesh::Pointer& expected)
{
    for (auto it = result->GetPoints()->Begin();
         it != result->GetPoints()->End(); ++it) {
        ITKPixel resultN;
        ITKPixel expectedN;
        result->GetPointData(it.Index(), &resultN);
        expected->GetPointData(it.Index(), &expectedN);

        EXPECT_DOUBLE_EQ(resultN[0], expectedN[0]);
        EXPECT_DOUBLE_EQ(resultN[1], expectedN[1]);
        EXPECT_DOUBLE_EQ(resultN[2], expectedN[2]);
    }
}

inline void ExpectNormalInverse(
    const ITKMesh::Pointer& result, const ITKMesh::Pointer& expected)
{
    for (auto it = result->GetPoints()->Begin();
         it != result->GetPoints()->End(); ++it) {
        ITKPixel resultN;
        ITKPixel expectedN;
        result->GetPointData(it.Index(), &resultN);
        expected->GetPointData(it.Index(), &expectedN);

        EXPECT_DOUBLE_EQ(resultN[0], -expectedN[0]);
        EXPECT_DOUBLE_EQ(resultN[1], -expectedN[1]);
        EXPECT_DOUBLE_EQ(resultN[2], -expectedN[2]);
    }
}

TEST(OrientNormals, Centroid)
{
    auto input = Cube().itkMesh();

    OrientNormals orient;
    orient.setReferenceMode(OrientNormals::ReferenceMode::Centroid);
    orient.setMesh(input);
    auto output = orient.compute();

    ExpectNormalInverse(output, input);
}

TEST(OrientNormals, ManualPositive)
{
    auto input = Plane().itkMesh();

    OrientNormals orient;
    orient.setReferencePoint({0, 1, 0});
    orient.setMesh(input);
    auto output = orient.compute();

    ExpectNormalEq(output, input);
}

TEST(OrientNormals, ManualNegative)
{
    auto input = Plane().itkMesh();

    OrientNormals orient;
    orient.setReferencePoint({0, -1, 0});
    orient.setMesh(input);
    auto output = orient.compute();

    ExpectNormalInverse(output, input);
}