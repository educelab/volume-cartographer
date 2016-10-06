//
// Created by Ryan Taber on 1/29/16.
//

#define BOOST_TEST_MODULE scaleMesh

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <common/io/objWriter.h>
#include "common/shapes/Arch.h"
#include "common/shapes/Cone.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Sphere.h"
#include "common/vc_defines.h"
#include "meshing/scaleMesh.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"

using namespace volcart;

/************************************************************************************
 *                                                                                  *
 *  scaleMeshTest.cpp - tests the functionality of /v-c/meshing/scaleMesh.cpp *
 *  The ultimate goal of this file is the following: *
 *                                                                                  *
 *        1. check whether an itk mesh can be scaled correctly without changing
 * *
 *           the input mesh. *
 *                                                                                  *
 *  This file is broken up into test fixtures for each of the common mesh shapes
 * *
 *  (plane, cube, cone, sphere, arch) which initialize objects for the following
 * *
 *  test cases: *
 *                                                                                  *
 *  1.  ScaledPlaneTest (ScaledPlaneFixture) *
 *  2.  ScaledCubeTest (ScaledCubeFixture) *
 *  3.  ScaledArchTest (ScaledArchFixture) *
 *  4.  ScaledSphereTest (ScaledSphereFixture) *
 *  5.  ScaledConeTest (ScaledConeFixture) *
 *  6.  ConfirmInputPlaneMeshIsUnchangedAfterScalingTest (ScaledPlaneFixture) *
 *  7.  CompareSavedAndFixtureScaledPlaneMesh (ScaledPlaneFixture) *
 *  8.  CompareSavedAndFixtureScaledCubeMesh (ScaledCubeFixture) *
 *  9.  CompareSavedAndFixtureScaledArchMesh (ScaledArchFixture) *
 *  10. CompareSavedAndFixtureScaledSphereMesh (ScaledSphereFixture) *
 *  11. CompareSavedAndFixtureScaledConeMesh (ScaledConeFixture) *
 *                                                                                  *
 * Input: *
 *     No required external inputs for this sample test. All test objects are *
 *     created internally by the various test fixtures. *
 *                                                                                  *
 * Test-Specific Output: *
 *     Specific test output only given on failure of any tests. Otherwise,
 * general  *
 *     number of testing errors is output. *
 *                                                                                  *
 * Miscellaneous: *
 *     See the /testing/meshing wiki for more information on this test *
 * **********************************************************************************/

/*
 * Individual test fixtures for each of the common shapes
 * Test cases call appropriate fixture
 */

struct ScaledPlaneFixture {

    ScaledPlaneFixture()
    {

        _in_PlaneMesh = _Plane.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_PlaneMeshUsedForRegressionTest = _Plane.itkMesh();
        volcart::meshing::scaleMesh(
            _in_PlaneMeshUsedForRegressionTest,
            _out_PlaneMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::parseObjFile(
            "ScaledPlaneMesh.obj", _SavedPlanePoints, _SavedPlaneCells);

        std::cerr << "setting up planar mesh for scaling" << std::endl;
    }

    ~ScaledPlaneFixture()
    {
        std::cerr << "cleaning up plane objects" << std::endl;
    }

    volcart::shapes::Plane _Plane;
    ITKMesh::Pointer _in_PlaneMesh, _in_PlaneMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_PlaneMesh = ITKMesh::New();
    double _ScaleFactor;

    // used for regression testing
    ITKMesh::Pointer _out_PlaneMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<Vertex> _SavedPlanePoints;
    std::vector<Cell> _SavedPlaneCells;
};

struct ScaledCubeFixture {

    ScaledCubeFixture()
    {

        _in_CubeMesh = _Cube.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_CubeMeshUsedForRegressionTest = _Cube.itkMesh();
        volcart::meshing::scaleMesh(
            _in_CubeMeshUsedForRegressionTest,
            _out_CubeMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::parseObjFile(
            "ScaledCubeMesh.obj", _SavedCubePoints, _SavedCubeCells);
        std::cerr << "setting up cube mesh for scaling" << std::endl;
    }

    ~ScaledCubeFixture()
    {
        std::cerr << "cleaning up cube objects" << std::endl;
    }

    ITKMesh::Pointer _in_CubeMesh, _in_CubeMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_CubeMesh = ITKMesh::New();
    volcart::shapes::Cube _Cube;
    double _ScaleFactor;

    ITKMesh::Pointer _out_CubeMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<Vertex> _SavedCubePoints;
    std::vector<Cell> _SavedCubeCells;
};

struct ScaledArchFixture {

    ScaledArchFixture()
    {

        std::cerr << "setting up arch mesh for scaling" << std::endl;

        _in_ArchMesh = _Arch.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_ArchMeshUsedForRegressionTest = _Arch.itkMesh();
        volcart::meshing::scaleMesh(
            _in_ArchMeshUsedForRegressionTest,
            _out_ArchMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::parseObjFile(
            "ScaledArchMesh.obj", _SavedArchPoints, _SavedArchCells);
    }

    ~ScaledArchFixture()
    {
        std::cerr << "cleaning up arch mesh objects" << std::endl;
    }

    ITKMesh::Pointer _in_ArchMesh, _in_ArchMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_ArchMesh = ITKMesh::New();
    volcart::shapes::Arch _Arch;
    double _ScaleFactor;

    ITKMesh::Pointer _out_ArchMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<Vertex> _SavedArchPoints;
    std::vector<Cell> _SavedArchCells;
};

struct ScaledSphereFixture {

    ScaledSphereFixture()
    {

        std::cerr << "setting up spherical mesh for scaling" << std::endl;

        _in_SphereMesh = _Sphere.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_SphereMeshUsedForRegressionTest = _Sphere.itkMesh();
        volcart::meshing::scaleMesh(
            _in_SphereMeshUsedForRegressionTest,
            _out_SphereMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::parseObjFile(
            "ScaledSphereMesh.obj", _SavedSpherePoints, _SavedSphereCells);
    }

    ~ScaledSphereFixture()
    {
        std::cerr << "cleaning up spherical mesh for scaling" << std::endl;
    }

    ITKMesh::Pointer _in_SphereMesh, _in_SphereMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_SphereMesh = ITKMesh::New();
    volcart::shapes::Sphere _Sphere;
    double _ScaleFactor;

    ITKMesh::Pointer _out_SphereMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<Vertex> _SavedSpherePoints;
    std::vector<Cell> _SavedSphereCells;
};

struct ScaledConeFixture {

    ScaledConeFixture()
    {

        _in_ConeMesh = _Cone.itkMesh();

        _ScaleFactor = 3;

        _in_ConeMeshUsedForRegressionTest = _Cone.itkMesh();
        volcart::meshing::scaleMesh(
            _in_ConeMeshUsedForRegressionTest,
            _out_ConeMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::parseObjFile(
            "ScaledConeMesh.obj", _SavedConePoints, _SavedConeCells);
        std::cerr << "setting up cone mesh for scaling" << std::endl;
    }

    ~ScaledConeFixture()
    {
        std::cerr << "cleaning up cone mesh for scaling" << std::endl;
    }

    ITKMesh::Pointer _in_ConeMesh, _in_ConeMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_ConeMesh = ITKMesh::New();
    volcart::shapes::Cone _Cone;
    double _ScaleFactor;

    ITKMesh::Pointer _out_ConeMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<Vertex> _SavedConePoints;
    std::vector<Cell> _SavedConeCells;
};

BOOST_FIXTURE_TEST_CASE(ScaledPlaneTest, ScaledPlaneFixture)
{

    // loop to cover positive, negative and zero value of factor
    while (_ScaleFactor > -1.5) {

        volcart::meshing::scaleMesh(
            _in_PlaneMesh, _out_PlaneMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(
            _in_PlaneMesh->GetNumberOfPoints(),
            _out_PlaneMesh->GetNumberOfPoints());

        for (size_t point = 0; point < _out_PlaneMesh->GetNumberOfPoints();
             ++point) {

            // check each of the points in the input and output meshes to
            // confirm
            // ratio matches scale factor

            volcart::testing::SmallOrClose(
                _in_PlaneMesh->GetPoint(point)[0] * _ScaleFactor,
                _out_PlaneMesh->GetPoint(point)[0]);
            volcart::testing::SmallOrClose(
                _in_PlaneMesh->GetPoint(point)[1] * _ScaleFactor,
                _out_PlaneMesh->GetPoint(point)[1]);
            volcart::testing::SmallOrClose(
                _in_PlaneMesh->GetPoint(point)[2] * _ScaleFactor,
                _out_PlaneMesh->GetPoint(point)[2]);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ScaledCubeTest, ScaledCubeFixture)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::scaleMesh(_in_CubeMesh, _out_CubeMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(
            _in_CubeMesh->GetNumberOfPoints(),
            _out_CubeMesh->GetNumberOfPoints());

        for (size_t point = 0; point < _out_CubeMesh->GetNumberOfPoints();
             ++point) {

            volcart::testing::SmallOrClose(
                _in_CubeMesh->GetPoint(point)[0] * _ScaleFactor,
                _out_CubeMesh->GetPoint(point)[0]);
            volcart::testing::SmallOrClose(
                _in_CubeMesh->GetPoint(point)[1] * _ScaleFactor,
                _out_CubeMesh->GetPoint(point)[1]);
            volcart::testing::SmallOrClose(
                _in_CubeMesh->GetPoint(point)[2] * _ScaleFactor,
                _out_CubeMesh->GetPoint(point)[2]);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ScaledArchTest, ScaledArchFixture)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::scaleMesh(_in_ArchMesh, _out_ArchMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(
            _in_ArchMesh->GetNumberOfPoints(),
            _out_ArchMesh->GetNumberOfPoints());

        for (size_t point = 0; point < _out_ArchMesh->GetNumberOfPoints();
             ++point) {

            volcart::testing::SmallOrClose(
                _in_ArchMesh->GetPoint(point)[0] * _ScaleFactor,
                _out_ArchMesh->GetPoint(point)[0], 0.0001);
            volcart::testing::SmallOrClose(
                _in_ArchMesh->GetPoint(point)[1] * _ScaleFactor,
                _out_ArchMesh->GetPoint(point)[1], 0.0001);
            volcart::testing::SmallOrClose(
                _in_ArchMesh->GetPoint(point)[2] * _ScaleFactor,
                _out_ArchMesh->GetPoint(point)[2], 0.0001);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ScaledSphereTest, ScaledSphereFixture)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::scaleMesh(
            _in_SphereMesh, _out_SphereMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(
            _in_SphereMesh->GetNumberOfPoints(),
            _out_SphereMesh->GetNumberOfPoints());

        for (size_t point = 0; point < _out_SphereMesh->GetNumberOfPoints();
             ++point) {

            volcart::testing::SmallOrClose(
                _in_SphereMesh->GetPoint(point)[0] * _ScaleFactor,
                _out_SphereMesh->GetPoint(point)[0], 0.0001);
            volcart::testing::SmallOrClose(
                _in_SphereMesh->GetPoint(point)[1] * _ScaleFactor,
                _out_SphereMesh->GetPoint(point)[1], 0.0001);
            volcart::testing::SmallOrClose(
                _in_SphereMesh->GetPoint(point)[2] * _ScaleFactor,
                _out_SphereMesh->GetPoint(point)[2], 0.0001);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(ScaledConeTest, ScaledConeFixture)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::scaleMesh(_in_ConeMesh, _out_ConeMesh, _ScaleFactor);

        BOOST_CHECK_EQUAL(
            _in_ConeMesh->GetNumberOfPoints(),
            _out_ConeMesh->GetNumberOfPoints());

        for (size_t point = 0; point < _out_ConeMesh->GetNumberOfPoints();
             ++point) {

            volcart::testing::SmallOrClose(
                _in_ConeMesh->GetPoint(point)[0] * _ScaleFactor,
                _out_ConeMesh->GetPoint(point)[0], 0.0001);
            volcart::testing::SmallOrClose(
                _in_ConeMesh->GetPoint(point)[1] * _ScaleFactor,
                _out_ConeMesh->GetPoint(point)[1], 0.0001);
            volcart::testing::SmallOrClose(
                _in_ConeMesh->GetPoint(point)[2] * _ScaleFactor,
                _out_ConeMesh->GetPoint(point)[2], 0.0001);
        }

        _ScaleFactor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(
    ConfirmInputPlaneMeshIsUnchangedAfterScalingTest, ScaledPlaneFixture)
{

    // reassign scale factor to a arbitrary value
    _ScaleFactor = 25;

    // call scaleMesh
    volcart::meshing::scaleMesh(_in_PlaneMesh, _out_PlaneMesh, _ScaleFactor);

    // init new plane mesh
    ITKMesh::Pointer NewPlaneMesh = _Plane.itkMesh();

    // compare _in_PlaneMesh and NewPlaneMesh to confirm _in_PlaneMesh goes
    // through
    // scaleMesh unchanged

    BOOST_CHECK_EQUAL(
        _in_PlaneMesh->GetNumberOfPoints(), NewPlaneMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(
        _in_PlaneMesh->GetNumberOfCells(), NewPlaneMesh->GetNumberOfCells());

    for (size_t point = 0; point < _in_PlaneMesh->GetNumberOfPoints();
         ++point) {

        BOOST_CHECK_EQUAL(
            _in_PlaneMesh->GetPoint(point)[0],
            NewPlaneMesh->GetPoint(point)[0]);
        BOOST_CHECK_EQUAL(
            _in_PlaneMesh->GetPoint(point)[1],
            NewPlaneMesh->GetPoint(point)[1]);
        BOOST_CHECK_EQUAL(
            _in_PlaneMesh->GetPoint(point)[2],
            NewPlaneMesh->GetPoint(point)[2]);
    }

    // Compare Cells
    ITKCellIterator in_PlaneMeshCell = _in_PlaneMesh->GetCells()->Begin();
    ITKCellIterator NewPlaneMeshCell = NewPlaneMesh->GetCells()->Begin();

    while (in_PlaneMeshCell != _in_PlaneMesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator in_PlaneMeshPointId =
            in_PlaneMeshCell.Value()->PointIdsBegin();
        ITKPointInCellIterator NewPlaneMeshPointId =
            NewPlaneMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (in_PlaneMeshPointId != in_PlaneMeshCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*in_PlaneMeshPointId, *NewPlaneMeshPointId);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(*in_PlaneMeshPointId, *NewPlaneMeshPointId);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*in_PlaneMeshPointId, *NewPlaneMeshPointId);

            // increment points
            ++in_PlaneMeshPointId;
            ++NewPlaneMeshPointId;
            ++counter;
        }

        // increment cells
        ++in_PlaneMeshCell;
        ++NewPlaneMeshCell;
    }
}

BOOST_FIXTURE_TEST_CASE(
    CompareSavedAndFixtureScaledPlaneMesh, ScaledPlaneFixture)
{

    BOOST_CHECK_EQUAL(
        _out_PlaneMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedPlanePoints.size());
    BOOST_CHECK_EQUAL(
        _out_PlaneMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedPlaneCells.size());

    // points
    for (size_t point = 0; point < _SavedPlanePoints.size(); ++point) {

        BOOST_CHECK_EQUAL(
            _out_PlaneMeshUsedForRegressionTest->GetPoint(point)[0],
            _SavedPlanePoints[point].x);
        BOOST_CHECK_EQUAL(
            _out_PlaneMeshUsedForRegressionTest->GetPoint(point)[1],
            _SavedPlanePoints[point].y);
        BOOST_CHECK_EQUAL(
            _out_PlaneMeshUsedForRegressionTest->GetPoint(point)[2],
            _SavedPlanePoints[point].z);
    }

    // Normals //
    int p = 0;
    ITKPointIterator point =
        _out_PlaneMeshUsedForRegressionTest->GetPoints()->Begin();
    for (; point != _out_PlaneMeshUsedForRegressionTest->GetPoints()->End();
         ++point) {

        ITKPixel _out_PlaneMeshUsedForRegressionTestNormal;
        _out_PlaneMeshUsedForRegressionTest->GetPointData(
            point.Index(), &_out_PlaneMeshUsedForRegressionTestNormal);

        // Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(
            _out_PlaneMeshUsedForRegressionTestNormal[0],
            _SavedPlanePoints[p].nx);
        BOOST_CHECK_EQUAL(
            _out_PlaneMeshUsedForRegressionTestNormal[1],
            _SavedPlanePoints[p].ny);
        BOOST_CHECK_EQUAL(
            _out_PlaneMeshUsedForRegressionTestNormal[2],
            _SavedPlanePoints[p].nz);

        ++p;
    }

    // Cells (faces)

    // Initialize Cell Iterator
    ITKCellIterator _out_PlaneMeshUsedForRegressionTestCell =
        _out_PlaneMeshUsedForRegressionTest->GetCells()->Begin();

    int c = 0;

    while (_out_PlaneMeshUsedForRegressionTestCell !=
           _out_PlaneMeshUsedForRegressionTest->GetCells()->End()) {

        // Initialize Iterator for Points in a Cell
        ITKPointInCellIterator _out_PlaneMeshUsedForRegressionTestPointId =
            _out_PlaneMeshUsedForRegressionTestCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (_out_PlaneMeshUsedForRegressionTestPointId !=
               _out_PlaneMeshUsedForRegressionTestCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(
                    *_out_PlaneMeshUsedForRegressionTestPointId,
                    _SavedPlaneCells[c].v1);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(
                    *_out_PlaneMeshUsedForRegressionTestPointId,
                    _SavedPlaneCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(
                    *_out_PlaneMeshUsedForRegressionTestPointId,
                    _SavedPlaneCells[c].v3);

            // increment points
            _out_PlaneMeshUsedForRegressionTestPointId++;
            counter++;
        }

        // increment cells
        ++_out_PlaneMeshUsedForRegressionTestCell;
        ++c;
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixtureScaledCubeMesh, ScaledCubeFixture)
{

    BOOST_CHECK_EQUAL(
        _out_CubeMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedCubePoints.size());
    BOOST_CHECK_EQUAL(
        _out_CubeMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedCubeCells.size());

    for (size_t point = 0; point < _SavedCubePoints.size(); ++point) {

        BOOST_CHECK_EQUAL(
            _out_CubeMeshUsedForRegressionTest->GetPoint(point)[0],
            _SavedCubePoints[point].x);
        BOOST_CHECK_EQUAL(
            _out_CubeMeshUsedForRegressionTest->GetPoint(point)[1],
            _SavedCubePoints[point].y);
        BOOST_CHECK_EQUAL(
            _out_CubeMeshUsedForRegressionTest->GetPoint(point)[2],
            _SavedCubePoints[point].z);
    }

    int p = 0;
    ITKPointIterator point =
        _out_CubeMeshUsedForRegressionTest->GetPoints()->Begin();
    for (; point != _out_CubeMeshUsedForRegressionTest->GetPoints()->End();
         ++point) {

        ITKPixel _out_CubeMeshUsedForRegressionTestNormal;
        _out_CubeMeshUsedForRegressionTest->GetPointData(
            point.Index(), &_out_CubeMeshUsedForRegressionTestNormal);

        BOOST_CHECK_EQUAL(
            _out_CubeMeshUsedForRegressionTestNormal[0],
            _SavedCubePoints[p].nx);
        BOOST_CHECK_EQUAL(
            _out_CubeMeshUsedForRegressionTestNormal[1],
            _SavedCubePoints[p].ny);
        BOOST_CHECK_EQUAL(
            _out_CubeMeshUsedForRegressionTestNormal[2],
            _SavedCubePoints[p].nz);

        ++p;
    }

    ITKCellIterator _out_CubeMeshUsedForRegressionTestCell =
        _out_CubeMeshUsedForRegressionTest->GetCells()->Begin();
    int c = 0;
    while (_out_CubeMeshUsedForRegressionTestCell !=
           _out_CubeMeshUsedForRegressionTest->GetCells()->End()) {

        ITKPointInCellIterator _out_CubeMeshUsedForRegressionTestPointId =
            _out_CubeMeshUsedForRegressionTestCell.Value()->PointIdsBegin();

        int counter = 0;
        while (_out_CubeMeshUsedForRegressionTestPointId !=
               _out_CubeMeshUsedForRegressionTestCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(
                    *_out_CubeMeshUsedForRegressionTestPointId,
                    _SavedCubeCells[c].v1);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(
                    *_out_CubeMeshUsedForRegressionTestPointId,
                    _SavedCubeCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(
                    *_out_CubeMeshUsedForRegressionTestPointId,
                    _SavedCubeCells[c].v3);

            _out_CubeMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_CubeMeshUsedForRegressionTestCell;
        ++c;
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixtureScaledArchMesh, ScaledArchFixture)
{

    BOOST_CHECK_EQUAL(
        _out_ArchMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedArchPoints.size());
    BOOST_CHECK_EQUAL(
        _out_ArchMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedArchCells.size());

    for (size_t point = 0; point < _SavedArchPoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_ArchMeshUsedForRegressionTest->GetPoint(point)[0],
            _SavedArchPoints[point].x);
        volcart::testing::SmallOrClose(
            _out_ArchMeshUsedForRegressionTest->GetPoint(point)[1],
            _SavedArchPoints[point].y);
        volcart::testing::SmallOrClose(
            _out_ArchMeshUsedForRegressionTest->GetPoint(point)[2],
            _SavedArchPoints[point].z);
    }

    int p = 0;
    ITKPointIterator point =
        _out_ArchMeshUsedForRegressionTest->GetPoints()->Begin();
    for (; point != _out_ArchMeshUsedForRegressionTest->GetPoints()->End();
         ++point) {

        ITKPixel _out_ArchMeshUsedForRegressionTestNormal;
        _out_ArchMeshUsedForRegressionTest->GetPointData(
            point.Index(), &_out_ArchMeshUsedForRegressionTestNormal);

        volcart::testing::SmallOrClose(
            _out_ArchMeshUsedForRegressionTestNormal[0],
            _SavedArchPoints[p].nx);
        volcart::testing::SmallOrClose(
            _out_ArchMeshUsedForRegressionTestNormal[1],
            _SavedArchPoints[p].ny);
        volcart::testing::SmallOrClose(
            _out_ArchMeshUsedForRegressionTestNormal[2],
            _SavedArchPoints[p].nz);

        ++p;
    }

    ITKCellIterator _out_ArchMeshUsedForRegressionTestCell =
        _out_ArchMeshUsedForRegressionTest->GetCells()->Begin();
    int c = 0;
    while (_out_ArchMeshUsedForRegressionTestCell !=
           _out_ArchMeshUsedForRegressionTest->GetCells()->End()) {

        ITKPointInCellIterator _out_ArchMeshUsedForRegressionTestPointId =
            _out_ArchMeshUsedForRegressionTestCell.Value()->PointIdsBegin();

        int counter = 0;
        while (_out_ArchMeshUsedForRegressionTestPointId !=
               _out_ArchMeshUsedForRegressionTestCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(
                    *_out_ArchMeshUsedForRegressionTestPointId,
                    _SavedArchCells[c].v1);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(
                    *_out_ArchMeshUsedForRegressionTestPointId,
                    _SavedArchCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(
                    *_out_ArchMeshUsedForRegressionTestPointId,
                    _SavedArchCells[c].v3);

            _out_ArchMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_ArchMeshUsedForRegressionTestCell;
        ++c;
    }
}

BOOST_FIXTURE_TEST_CASE(
    CompareSavedAndFixtureScaledSphereMesh, ScaledSphereFixture)
{

    BOOST_CHECK_EQUAL(
        _out_SphereMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedSpherePoints.size());
    BOOST_CHECK_EQUAL(
        _out_SphereMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedSphereCells.size());

    for (size_t point = 0; point < _SavedSpherePoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_SphereMeshUsedForRegressionTest->GetPoint(point)[0],
            _SavedSpherePoints[point].x);
        volcart::testing::SmallOrClose(
            _out_SphereMeshUsedForRegressionTest->GetPoint(point)[1],
            _SavedSpherePoints[point].y);
        volcart::testing::SmallOrClose(
            _out_SphereMeshUsedForRegressionTest->GetPoint(point)[2],
            _SavedSpherePoints[point].z);
    }

    int p = 0;
    ITKPointIterator point =
        _out_SphereMeshUsedForRegressionTest->GetPoints()->Begin();
    for (; point != _out_SphereMeshUsedForRegressionTest->GetPoints()->End();
         ++point) {

        ITKPixel _out_SphereMeshUsedForRegressionTestNormal;
        _out_SphereMeshUsedForRegressionTest->GetPointData(
            point.Index(), &_out_SphereMeshUsedForRegressionTestNormal);

        volcart::testing::SmallOrClose(
            _out_SphereMeshUsedForRegressionTestNormal[0],
            _SavedSpherePoints[p].nx);
        volcart::testing::SmallOrClose(
            _out_SphereMeshUsedForRegressionTestNormal[1],
            _SavedSpherePoints[p].ny);
        volcart::testing::SmallOrClose(
            _out_SphereMeshUsedForRegressionTestNormal[2],
            _SavedSpherePoints[p].nz);

        ++p;
    }

    ITKCellIterator _out_SphereMeshUsedForRegressionTestCell =
        _out_SphereMeshUsedForRegressionTest->GetCells()->Begin();
    int c = 0;
    while (_out_SphereMeshUsedForRegressionTestCell !=
           _out_SphereMeshUsedForRegressionTest->GetCells()->End()) {

        ITKPointInCellIterator _out_SphereMeshUsedForRegressionTestPointId =
            _out_SphereMeshUsedForRegressionTestCell.Value()->PointIdsBegin();

        int counter = 0;
        while (
            _out_SphereMeshUsedForRegressionTestPointId !=
            _out_SphereMeshUsedForRegressionTestCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(
                    *_out_SphereMeshUsedForRegressionTestPointId,
                    _SavedSphereCells[c].v1);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(
                    *_out_SphereMeshUsedForRegressionTestPointId,
                    _SavedSphereCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(
                    *_out_SphereMeshUsedForRegressionTestPointId,
                    _SavedSphereCells[c].v3);

            _out_SphereMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_SphereMeshUsedForRegressionTestCell;
        ++c;
    }
}

BOOST_FIXTURE_TEST_CASE(CompareSavedAndFixtureScaledConeMesh, ScaledConeFixture)
{

    BOOST_CHECK_EQUAL(
        _out_ConeMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedConePoints.size());
    BOOST_CHECK_EQUAL(
        _out_ConeMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedConeCells.size());

    for (size_t point = 0; point < _SavedConePoints.size(); ++point) {

        volcart::testing::SmallOrClose(
            _out_ConeMeshUsedForRegressionTest->GetPoint(point)[0],
            _SavedConePoints[point].x);
        volcart::testing::SmallOrClose(
            _out_ConeMeshUsedForRegressionTest->GetPoint(point)[1],
            _SavedConePoints[point].y);
        volcart::testing::SmallOrClose(
            _out_ConeMeshUsedForRegressionTest->GetPoint(point)[2],
            _SavedConePoints[point].z);
    }

    int p = 0;
    ITKPointIterator point =
        _out_ConeMeshUsedForRegressionTest->GetPoints()->Begin();
    for (; point != _out_ConeMeshUsedForRegressionTest->GetPoints()->End();
         ++point) {

        ITKPixel _out_ConeMeshUsedForRegressionTestNormal;
        _out_ConeMeshUsedForRegressionTest->GetPointData(
            point.Index(), &_out_ConeMeshUsedForRegressionTestNormal);

        volcart::testing::SmallOrClose(
            _out_ConeMeshUsedForRegressionTestNormal[0],
            _SavedConePoints[p].nx);
        volcart::testing::SmallOrClose(
            _out_ConeMeshUsedForRegressionTestNormal[1],
            _SavedConePoints[p].ny);
        volcart::testing::SmallOrClose(
            _out_ConeMeshUsedForRegressionTestNormal[2],
            _SavedConePoints[p].nz);

        ++p;
    }

    ITKCellIterator _out_ConeMeshUsedForRegressionTestCell =
        _out_ConeMeshUsedForRegressionTest->GetCells()->Begin();
    int c = 0;
    while (_out_ConeMeshUsedForRegressionTestCell !=
           _out_ConeMeshUsedForRegressionTest->GetCells()->End()) {

        ITKPointInCellIterator _out_ConeMeshUsedForRegressionTestPointId =
            _out_ConeMeshUsedForRegressionTestCell.Value()->PointIdsBegin();

        int counter = 0;
        while (_out_ConeMeshUsedForRegressionTestPointId !=
               _out_ConeMeshUsedForRegressionTestCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(
                    *_out_ConeMeshUsedForRegressionTestPointId,
                    _SavedConeCells[c].v1);
            else if (counter == 1)
                BOOST_CHECK_EQUAL(
                    *_out_ConeMeshUsedForRegressionTestPointId,
                    _SavedConeCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(
                    *_out_ConeMeshUsedForRegressionTestPointId,
                    _SavedConeCells[c].v3);

            _out_ConeMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_ConeMeshUsedForRegressionTestCell;
        ++c;
    }
}
