//
// Created by Hannah Hatch on 7/26/16.
//

#define BOOST_TEST_MODULE CalculateNormals

#include <boost/test/unit_test.hpp>

#include "vc/core/shapes/Plane.hpp"
#include "vc/meshing/CalculateNormals.hpp"
#include "vc/testing/TestingUtils.hpp"

struct PlaneFixture {
    PlaneFixture() { _in_Mesh = _Plane.itkMesh(); }
    ~PlaneFixture() {}

    volcart::shapes::Plane _Plane;
    volcart::ITKMesh::Pointer _in_Mesh, _out_Mesh;
};

BOOST_FIXTURE_TEST_CASE(ComputePlaneNormalsTest, PlaneFixture)
{

    volcart::meshing::CalculateNormals calcNorm(_in_Mesh);
    calcNorm.compute();
    _out_Mesh = calcNorm.getMesh();

    for (auto p_it = _out_Mesh->GetPoints()->Begin();
         p_it != _out_Mesh->GetPoints()->End(); ++p_it) {
        volcart::ITKPixel out_Normal, in_Normal;
        _out_Mesh->GetPointData(p_it.Index(), &out_Normal);
        _in_Mesh->GetPointData(p_it.Index(), &in_Normal);

        volcart::testing::SmallOrClose(out_Normal[0], in_Normal[0]);
        volcart::testing::SmallOrClose(out_Normal[1], in_Normal[1]);
        volcart::testing::SmallOrClose(out_Normal[2], in_Normal[2]);
    }
}
