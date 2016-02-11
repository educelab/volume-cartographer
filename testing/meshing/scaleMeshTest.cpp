//
// Created by Ryan Taber on 1/29/16.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE scaleMesh

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <io/objWriter.h>
#include "vc_defines.h"
#include "shapes.h"
#include "scaleMesh.h"

/************************************************************************************
 *                                                                                  *
 *  scaleMeshTest.cpp - tests the functionality of /v-c/meshing/scaleMesh.cpp       *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether an itk mesh can be scaled correctly without changing     *
 *           the input mesh.                                                        *
 *                                                                                  *
 *  This file is broken up into test fixtures for each of the common mesh shapes    *
 *  (plane, cube, cone, sphere, arch) which initialize objects for the following    *
 *  test cases:                                                                     *
 *                                                                                  *
 *  1. ScaledPlaneTest (PlaneFixture)                                               *
 *  2. ScaledCubeTest (CubeFixture)                                                 *
 *  3. ScaledArchTest (ArchFixture)                                                 *
 *  4. ScaledSphereTest (SphereFixture)                                             *
 *  5. ScaledConeTest (ConeFixture)                                                 *
 *  6. ConfirmInputPlaneMeshIsUnchangedAfterScalingTest (PlaneFixture)              *
 *                                                                                  *
 * Input:                                                                           *
 *     No required external inputs for this sample test. All test objects are       *
 *     created internally by the various test fixtures.                             *
 *                                                                                  *
 * Test-Specific Output:                                                            *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output.                                          *
 *                                                                                  *
 * Miscellaneous:                                                                   *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/


/*
 * Individual test fixtures for each of the common shapes
 * Test cases call appropriate fixture
 */

struct PlaneFixture {

    PlaneFixture() {

        _in_PlaneMesh = _Plane.itkMesh();

        _ScaleFactor = 3;

        std::cerr << "setting up planar mesh for scaling" << std::endl;
    }

    ~PlaneFixture(){ std::cerr << "cleaning up plane objects" << std::endl; }

    VC_MeshType::Pointer _in_PlaneMesh;
    VC_MeshType::Pointer _out_PlaneMesh = VC_MeshType::New();
    volcart::shapes::Plane _Plane;
    double _ScaleFactor;
};

struct CubeFixture {

    CubeFixture() {

        _in_CubeMesh = _Cube.itkMesh();

        _ScaleFactor = 3;

        std::cerr << "setting up cube mesh for scaling" << std::endl;
    }

    ~CubeFixture(){ std::cerr << "cleaning up cube objects" << std::endl; }

    VC_MeshType::Pointer _in_CubeMesh;
    VC_MeshType::Pointer _out_CubeMesh = VC_MeshType::New();
    volcart::shapes::Cube _Cube;
    double _ScaleFactor;
};

struct ArchFixture {

    ArchFixture() {

        _in_ArchMesh = _Arch.itkMesh();

        _ScaleFactor = 3;

        std::cerr << "setting up arch mesh for scaling" << std::endl;
    }

    ~ArchFixture(){ std::cerr << "cleaning up arch mesh objects" << std::endl; }

    VC_MeshType::Pointer _in_ArchMesh;
    VC_MeshType::Pointer _out_ArchMesh = VC_MeshType::New();
    volcart::shapes::Arch _Arch;
    double _ScaleFactor;
};

struct SphereFixture {

    SphereFixture() {

        _in_SphereMesh = _Sphere.itkMesh();

        _ScaleFactor = 3;

        std::cerr << "setting up spherical mesh for scaling" << std::endl;
    }

    ~SphereFixture(){ std::cerr << "cleaning up spherical mesh for scaling" << std::endl; }

    VC_MeshType::Pointer _in_SphereMesh;
    VC_MeshType::Pointer _out_SphereMesh = VC_MeshType::New();
    volcart::shapes::Sphere _Sphere;
    double _ScaleFactor;
};

struct ConeFixture {

    ConeFixture() {

        _in_ConeMesh = _Cone.itkMesh();

        _ScaleFactor = 3;

        std::cerr << "setting up cone mesh for scaling" << std::endl;
    }

    ~ConeFixture(){ std::cerr << "cleaning up cone mesh for scaling" << std::endl; }

    VC_MeshType::Pointer _in_ConeMesh;
    VC_MeshType::Pointer _out_ConeMesh = VC_MeshType::New();
    volcart::shapes::Cone _Cone;
    double _ScaleFactor;
};


BOOST_FIXTURE_TEST_CASE(ScaledPlaneTest, PlaneFixture){


    //loop to cover positive, negative and zero value of factor
    while (_ScaleFactor > -1.5){

        volcart::meshing::scaleMesh(_in_PlaneMesh, _out_PlaneMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(_in_PlaneMesh->GetNumberOfPoints(), _out_PlaneMesh->GetNumberOfPoints());


        for (size_t point; point < _out_PlaneMesh->GetNumberOfPoints(); ++point){

            //check each of the points in the input and output meshes to confirm
            //ratio matches scale factor

            BOOST_CHECK_EQUAL(_in_PlaneMesh->GetPoint(point)[0] * _ScaleFactor, _out_PlaneMesh->GetPoint(point)[0]);
            BOOST_CHECK_EQUAL(_in_PlaneMesh->GetPoint(point)[1] * _ScaleFactor, _out_PlaneMesh->GetPoint(point)[1]);
            BOOST_CHECK_EQUAL(_in_PlaneMesh->GetPoint(point)[2] * _ScaleFactor, _out_PlaneMesh->GetPoint(point)[2]);

        }

        _ScaleFactor -= 0.5;
    }

}

BOOST_FIXTURE_TEST_CASE(ScaledCubeTest, CubeFixture){

    while (_ScaleFactor > -1.5){

        volcart::meshing::scaleMesh(_in_CubeMesh, _out_CubeMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(_in_CubeMesh->GetNumberOfPoints(), _out_CubeMesh->GetNumberOfPoints());


        for (size_t point; point < _out_CubeMesh->GetNumberOfPoints(); ++point){

            BOOST_CHECK_EQUAL(_in_CubeMesh->GetPoint(point)[0] * _ScaleFactor, _out_CubeMesh->GetPoint(point)[0]);
            BOOST_CHECK_EQUAL(_in_CubeMesh->GetPoint(point)[1] * _ScaleFactor, _out_CubeMesh->GetPoint(point)[1]);
            BOOST_CHECK_EQUAL(_in_CubeMesh->GetPoint(point)[2] * _ScaleFactor, _out_CubeMesh->GetPoint(point)[2]);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ScaledArchTest, ArchFixture){

    while (_ScaleFactor > -1.5){

        volcart::meshing::scaleMesh(_in_ArchMesh, _out_ArchMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(_in_ArchMesh->GetNumberOfPoints(), _out_ArchMesh->GetNumberOfPoints());

        for (size_t point; point < _out_ArchMesh->GetNumberOfPoints(); ++point){

            BOOST_CHECK_EQUAL(_in_ArchMesh->GetPoint(point)[0] * _ScaleFactor, _out_ArchMesh->GetPoint(point)[0]);
            BOOST_CHECK_EQUAL(_in_ArchMesh->GetPoint(point)[1] * _ScaleFactor, _out_ArchMesh->GetPoint(point)[1]);
            BOOST_CHECK_EQUAL(_in_ArchMesh->GetPoint(point)[2] * _ScaleFactor, _out_ArchMesh->GetPoint(point)[2]);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ScaledSphereTest, SphereFixture){

    while (_ScaleFactor > -1.5){

        volcart::meshing::scaleMesh(_in_SphereMesh, _out_SphereMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(_in_SphereMesh->GetNumberOfPoints(), _out_SphereMesh->GetNumberOfPoints());

        for (size_t point; point < _out_SphereMesh->GetNumberOfPoints(); ++point){

            BOOST_CHECK_EQUAL(_in_SphereMesh->GetPoint(point)[0] * _ScaleFactor, _out_SphereMesh->GetPoint(point)[0]);
            BOOST_CHECK_EQUAL(_in_SphereMesh->GetPoint(point)[1] * _ScaleFactor, _out_SphereMesh->GetPoint(point)[1]);
            BOOST_CHECK_EQUAL(_in_SphereMesh->GetPoint(point)[2] * _ScaleFactor, _out_SphereMesh->GetPoint(point)[2]);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ScaledConeTest, ConeFixture){

    while (_ScaleFactor > -1.5){

        volcart::meshing::scaleMesh(_in_ConeMesh, _out_ConeMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(_in_ConeMesh->GetNumberOfPoints(), _out_ConeMesh->GetNumberOfPoints());

        for (size_t point; point < _out_ConeMesh->GetNumberOfPoints(); ++point){

            BOOST_CHECK_EQUAL(_in_ConeMesh->GetPoint(point)[0] * _ScaleFactor, _out_ConeMesh->GetPoint(point)[0]);
            BOOST_CHECK_EQUAL(_in_ConeMesh->GetPoint(point)[1] * _ScaleFactor, _out_ConeMesh->GetPoint(point)[1]);
            BOOST_CHECK_EQUAL(_in_ConeMesh->GetPoint(point)[2] * _ScaleFactor, _out_ConeMesh->GetPoint(point)[2]);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ConfirmInputPlaneMeshIsUnchangedAfterScalingTest, PlaneFixture){

    //reassign scale factor to a larger value
    _ScaleFactor = 25;

    //call scaleMesh
    volcart::meshing::scaleMesh(_in_PlaneMesh, _out_PlaneMesh, _ScaleFactor);

    //init new plane mesh
    VC_MeshType::Pointer NewPlaneMesh = _Plane.itkMesh();

    //compare _in_PlaneMesh and NewPlaneMesh to confirm _in_PlaneMesh goes through
    //scaleMesh unchanged

    BOOST_CHECK_EQUAL(_in_PlaneMesh->GetNumberOfPoints(), NewPlaneMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(_in_PlaneMesh->GetNumberOfCells(), NewPlaneMesh->GetNumberOfCells());

    for (size_t point; point < _in_PlaneMesh->GetNumberOfPoints(); ++point){

        BOOST_CHECK_EQUAL(_in_PlaneMesh->GetPoint(point)[0], NewPlaneMesh->GetPoint(point)[0]);
        BOOST_CHECK_EQUAL(_in_PlaneMesh->GetPoint(point)[1], NewPlaneMesh->GetPoint(point)[1]);
        BOOST_CHECK_EQUAL(_in_PlaneMesh->GetPoint(point)[2], NewPlaneMesh->GetPoint(point)[2]);

    }

    // Compare Cells
    VC_CellIterator in_PlaneMeshCell = _in_PlaneMesh->GetCells()->Begin();
    VC_CellIterator NewPlaneMeshCell = NewPlaneMesh->GetCells()->Begin();

    while (in_PlaneMeshCell != _in_PlaneMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator in_PlaneMeshPointId = in_PlaneMeshCell.Value()->PointIdsBegin();
        VC_PointsInCellIterator NewPlaneMeshPointId = NewPlaneMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( in_PlaneMeshPointId != in_PlaneMeshCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*in_PlaneMeshPointId, *NewPlaneMeshPointId);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*in_PlaneMeshPointId, *NewPlaneMeshPointId);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*in_PlaneMeshPointId, *NewPlaneMeshPointId);

            //increment points
            ++in_PlaneMeshPointId; ++NewPlaneMeshPointId; ++counter;
        }

        //increment cells
        ++in_PlaneMeshCell; ++NewPlaneMeshCell;
    }
}
