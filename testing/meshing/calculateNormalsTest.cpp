//
// Created by Hannah Hatch on 7/26/16.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE orderedResampling

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <calculateNormals.h>
#include "vc_defines.h"
#include "calculateNormals.h"
#include "shapes.h"

struct PlaneFixture {
    PlaneFixture(){
        _in_PlaneMesh = _Plane.itkMesh();

    }
    volcart::shapes::Plane _Plane;
    VC_MeshType::Pointer _in_PlaneMesh, _out_PlaneMesh;
};

BOOST_FIXTURE_TEST_CASE(computePlaneNormalsTest, PlaneFixture){
    volcart::meshing::calculateNormals input(_in_PlaneMesh);
//    input.computeNormals();
//    _out_PlaneMesh = input.getOutput();

//    VC_PointsInMeshIterator in_point = _in_PlaneMesh->GetPoints()->Begin();
//    VC_PointsInMeshIterator out_point = _out_PlaneMesh->GetPoints()->Begin();
//    for(; out_point != _out_PlaneMesh->GetPoints()->End(); ++out_point)
//    {
//        VC_PixelType out_PlaneNormal, in_PlaneNormal;
//        _out_PlaneMesh->GetPointData(out_point.Index(), &out_PlaneNormal);
//        _in_PlaneMesh->GetPointData(in_point.Index(), &in_PlaneNormal);
//
//        BOOST_CHECK_EQUAL(out_PlaneNormal[0], in_PlaneNormal[0]);
//        BOOST_CHECK_EQUAL(out_PlaneNormal[1], in_PlaneNormal[1]);
//        BOOST_CHECK_EQUAL(out_PlaneNormal[2], in_PlaneNormal[2]);
//
//        ++in_point;
//    }
}