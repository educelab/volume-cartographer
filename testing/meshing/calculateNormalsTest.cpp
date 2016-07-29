//
// Created by Hannah Hatch on 7/26/16.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE calculateNormals

#include <boost/test/unit_test.hpp>

#include "vc_defines.h"
#include "calculateNormals.h"
#include "shapes.h"

struct PlaneFixture {
    PlaneFixture() {
        _in_Mesh = _Plane.itkMesh();
    }
    ~PlaneFixture(){};

    volcart::shapes::Plane _Plane;
    VC_MeshType::Pointer _in_Mesh, _out_Mesh;
};

BOOST_FIXTURE_TEST_CASE(ComputePlaneNormalsTest, PlaneFixture){

    volcart::meshing::calculateNormals calcNorm( _in_Mesh );
    calcNorm.compute();
    _out_Mesh = calcNorm.getMesh();

    for(auto p_it = _out_Mesh->GetPoints()->Begin(); p_it != _out_Mesh->GetPoints()->End(); ++p_it)
    {
        VC_PixelType out_Normal, in_Normal;
        _out_Mesh->GetPointData(p_it.Index(), &out_Normal);
        _in_Mesh->GetPointData(p_it.Index(), &in_Normal);

        BOOST_CHECK_EQUAL(out_Normal[0], in_Normal[0]);
        BOOST_CHECK_EQUAL(out_Normal[1], in_Normal[1]);
        BOOST_CHECK_EQUAL(out_Normal[2], in_Normal[2]);
    }
}