#include <gtest/gtest.h>

#include <cstddef>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Cone.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Sphere.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/meshing/ScaleMesh.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;

/*
 * Individual test fixtures for each of the common shapes
 * Test cases call appropriate fixture
 */

class ScaledPlaneFixture : public ::testing::Test
{
public:
    ScaledPlaneFixture()
    {

        _in_PlaneMesh = _Plane.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_PlaneMeshUsedForRegressionTest = _Plane.itkMesh();
        volcart::meshing::ScaleMesh(
            _in_PlaneMeshUsedForRegressionTest,
            _out_PlaneMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "ScaledPlaneMesh.obj", _SavedPlanePoints, _SavedPlaneCells);
    }

    volcart::shapes::Plane _Plane;
    ITKMesh::Pointer _in_PlaneMesh, _in_PlaneMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_PlaneMesh = ITKMesh::New();
    double _ScaleFactor;

    // used for regression testing
    ITKMesh::Pointer _out_PlaneMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<SimpleMesh::Vertex> _SavedPlanePoints;
    std::vector<SimpleMesh::Cell> _SavedPlaneCells;
};

class ScaledCubeFixture : public ::testing::Test
{
public:
    ScaledCubeFixture()
    {

        _in_CubeMesh = _Cube.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_CubeMeshUsedForRegressionTest = _Cube.itkMesh();
        volcart::meshing::ScaleMesh(
            _in_CubeMeshUsedForRegressionTest,
            _out_CubeMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "ScaledCubeMesh.obj", _SavedCubePoints, _SavedCubeCells);
    }

    ITKMesh::Pointer _in_CubeMesh, _in_CubeMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_CubeMesh = ITKMesh::New();
    volcart::shapes::Cube _Cube;
    double _ScaleFactor;

    ITKMesh::Pointer _out_CubeMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<SimpleMesh::Vertex> _SavedCubePoints;
    std::vector<SimpleMesh::Cell> _SavedCubeCells;
};

class ScaledArchFixture : public ::testing::Test
{
public:
    ScaledArchFixture()
    {
        _in_ArchMesh = _Arch.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_ArchMeshUsedForRegressionTest = _Arch.itkMesh();
        volcart::meshing::ScaleMesh(
            _in_ArchMeshUsedForRegressionTest,
            _out_ArchMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "ScaledArchMesh.obj", _SavedArchPoints, _SavedArchCells);
    }

    ITKMesh::Pointer _in_ArchMesh, _in_ArchMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_ArchMesh = ITKMesh::New();
    volcart::shapes::Arch _Arch;
    double _ScaleFactor;

    ITKMesh::Pointer _out_ArchMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<SimpleMesh::Vertex> _SavedArchPoints;
    std::vector<SimpleMesh::Cell> _SavedArchCells;
};

class ScaledSphereFixture : public ::testing::Test
{
public:
    ScaledSphereFixture()
    {
        _in_SphereMesh = _Sphere.itkMesh();
        _ScaleFactor = 3;

        // Regression
        _in_SphereMeshUsedForRegressionTest = _Sphere.itkMesh();
        volcart::meshing::ScaleMesh(
            _in_SphereMeshUsedForRegressionTest,
            _out_SphereMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "ScaledSphereMesh.obj", _SavedSpherePoints, _SavedSphereCells);
    }

    ITKMesh::Pointer _in_SphereMesh, _in_SphereMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_SphereMesh = ITKMesh::New();
    volcart::shapes::Sphere _Sphere;
    double _ScaleFactor;

    ITKMesh::Pointer _out_SphereMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<SimpleMesh::Vertex> _SavedSpherePoints;
    std::vector<SimpleMesh::Cell> _SavedSphereCells;
};

class ScaledConeFixture : public ::testing::Test
{
public:
    ScaledConeFixture()
    {

        _in_ConeMesh = _Cone.itkMesh();

        _ScaleFactor = 3;

        _in_ConeMeshUsedForRegressionTest = _Cone.itkMesh();
        volcart::meshing::ScaleMesh(
            _in_ConeMeshUsedForRegressionTest,
            _out_ConeMeshUsedForRegressionTest, 3);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "ScaledConeMesh.obj", _SavedConePoints, _SavedConeCells);
    }

    ITKMesh::Pointer _in_ConeMesh, _in_ConeMeshUsedForRegressionTest;
    ITKMesh::Pointer _out_ConeMesh = ITKMesh::New();
    volcart::shapes::Cone _Cone;
    double _ScaleFactor;

    ITKMesh::Pointer _out_ConeMeshUsedForRegressionTest = ITKMesh::New();
    std::vector<SimpleMesh::Vertex> _SavedConePoints;
    std::vector<SimpleMesh::Cell> _SavedConeCells;
};

TEST_F(ScaledPlaneFixture, ScaledPlaneTest)
{

    // loop to cover positive, negative and zero value of factor
    while (_ScaleFactor > -1.5) {

        volcart::meshing::ScaleMesh(
            _in_PlaneMesh, _out_PlaneMesh, _ScaleFactor);

        EXPECT_EQ(
            _in_PlaneMesh->GetNumberOfPoints(),
            _out_PlaneMesh->GetNumberOfPoints());

        for (std::size_t point = 0; point < _out_PlaneMesh->GetNumberOfPoints();
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

TEST_F(ScaledCubeFixture, ScaledCubeTest)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::ScaleMesh(_in_CubeMesh, _out_CubeMesh, _ScaleFactor);

        EXPECT_EQ(
            _in_CubeMesh->GetNumberOfPoints(),
            _out_CubeMesh->GetNumberOfPoints());

        for (std::size_t point = 0; point < _out_CubeMesh->GetNumberOfPoints();
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

TEST_F(ScaledArchFixture, ScaledArchTest)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::ScaleMesh(_in_ArchMesh, _out_ArchMesh, _ScaleFactor);

        EXPECT_EQ(
            _in_ArchMesh->GetNumberOfPoints(),
            _out_ArchMesh->GetNumberOfPoints());

        for (std::size_t point = 0; point < _out_ArchMesh->GetNumberOfPoints();
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

TEST_F(ScaledSphereFixture, ScaledSphereTest)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::ScaleMesh(
            _in_SphereMesh, _out_SphereMesh, _ScaleFactor);

        EXPECT_EQ(
            _in_SphereMesh->GetNumberOfPoints(),
            _out_SphereMesh->GetNumberOfPoints());

        for (std::size_t point = 0;
             point < _out_SphereMesh->GetNumberOfPoints(); ++point) {

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

TEST_F(ScaledConeFixture, ScaledConeTest)
{

    while (_ScaleFactor > -1.5) {

        volcart::meshing::ScaleMesh(_in_ConeMesh, _out_ConeMesh, _ScaleFactor);

        EXPECT_EQ(
            _in_ConeMesh->GetNumberOfPoints(),
            _out_ConeMesh->GetNumberOfPoints());

        for (std::size_t point = 0; point < _out_ConeMesh->GetNumberOfPoints();
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

TEST_F(ScaledPlaneFixture, ConfirmInputPlaneMeshIsUnchangedAfterScalingTest)
{

    // reassign scale factor to a arbitrary value
    _ScaleFactor = 25;

    // call ScaleMesh
    volcart::meshing::ScaleMesh(_in_PlaneMesh, _out_PlaneMesh, _ScaleFactor);

    // init new plane mesh
    ITKMesh::Pointer NewPlaneMesh = _Plane.itkMesh();

    // compare _in_PlaneMesh and NewPlaneMesh to confirm _in_PlaneMesh goes
    // through
    // ScaleMesh unchanged

    EXPECT_EQ(
        _in_PlaneMesh->GetNumberOfPoints(), NewPlaneMesh->GetNumberOfPoints());
    EXPECT_EQ(
        _in_PlaneMesh->GetNumberOfCells(), NewPlaneMesh->GetNumberOfCells());

    for (std::size_t point = 0; point < _in_PlaneMesh->GetNumberOfPoints();
         ++point) {

        EXPECT_EQ(
            _in_PlaneMesh->GetPoint(point)[0],
            NewPlaneMesh->GetPoint(point)[0]);
        EXPECT_EQ(
            _in_PlaneMesh->GetPoint(point)[1],
            NewPlaneMesh->GetPoint(point)[1]);
        EXPECT_EQ(
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
                EXPECT_EQ(*in_PlaneMeshPointId, *NewPlaneMeshPointId);
            else if (counter == 1)
                EXPECT_EQ(*in_PlaneMeshPointId, *NewPlaneMeshPointId);
            else if (counter == 2)
                EXPECT_EQ(*in_PlaneMeshPointId, *NewPlaneMeshPointId);

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

TEST_F(ScaledPlaneFixture, CompareSavedAndFixtureScaledPlaneMesh)
{

    EXPECT_EQ(
        _out_PlaneMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedPlanePoints.size());
    EXPECT_EQ(
        _out_PlaneMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedPlaneCells.size());

    // points
    for (std::size_t point = 0; point < _SavedPlanePoints.size(); ++point) {

        EXPECT_EQ(
            _out_PlaneMeshUsedForRegressionTest->GetPoint(point)[0],
            _SavedPlanePoints[point].x);
        EXPECT_EQ(
            _out_PlaneMeshUsedForRegressionTest->GetPoint(point)[1],
            _SavedPlanePoints[point].y);
        EXPECT_EQ(
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
        EXPECT_EQ(
            _out_PlaneMeshUsedForRegressionTestNormal[0],
            _SavedPlanePoints[p].nx);
        EXPECT_EQ(
            _out_PlaneMeshUsedForRegressionTestNormal[1],
            _SavedPlanePoints[p].ny);
        EXPECT_EQ(
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
                EXPECT_EQ(
                    *_out_PlaneMeshUsedForRegressionTestPointId,
                    _SavedPlaneCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(
                    *_out_PlaneMeshUsedForRegressionTestPointId,
                    _SavedPlaneCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(
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

TEST_F(ScaledCubeFixture, CompareSavedAndFixtureScaledCubeMesh)
{

    EXPECT_EQ(
        _out_CubeMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedCubePoints.size());
    EXPECT_EQ(
        _out_CubeMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedCubeCells.size());

    for (std::size_t point = 0; point < _SavedCubePoints.size(); ++point) {

        EXPECT_EQ(
            _out_CubeMeshUsedForRegressionTest->GetPoint(point)[0],
            _SavedCubePoints[point].x);
        EXPECT_EQ(
            _out_CubeMeshUsedForRegressionTest->GetPoint(point)[1],
            _SavedCubePoints[point].y);
        EXPECT_EQ(
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

        EXPECT_EQ(
            _out_CubeMeshUsedForRegressionTestNormal[0],
            _SavedCubePoints[p].nx);
        EXPECT_EQ(
            _out_CubeMeshUsedForRegressionTestNormal[1],
            _SavedCubePoints[p].ny);
        EXPECT_EQ(
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
                EXPECT_EQ(
                    *_out_CubeMeshUsedForRegressionTestPointId,
                    _SavedCubeCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(
                    *_out_CubeMeshUsedForRegressionTestPointId,
                    _SavedCubeCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(
                    *_out_CubeMeshUsedForRegressionTestPointId,
                    _SavedCubeCells[c].v3);

            _out_CubeMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_CubeMeshUsedForRegressionTestCell;
        ++c;
    }
}

TEST_F(ScaledArchFixture, CompareSavedAndFixtureScaledArchMesh)
{

    EXPECT_EQ(
        _out_ArchMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedArchPoints.size());
    EXPECT_EQ(
        _out_ArchMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedArchCells.size());

    for (std::size_t point = 0; point < _SavedArchPoints.size(); ++point) {

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
                EXPECT_EQ(
                    *_out_ArchMeshUsedForRegressionTestPointId,
                    _SavedArchCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(
                    *_out_ArchMeshUsedForRegressionTestPointId,
                    _SavedArchCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(
                    *_out_ArchMeshUsedForRegressionTestPointId,
                    _SavedArchCells[c].v3);

            _out_ArchMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_ArchMeshUsedForRegressionTestCell;
        ++c;
    }
}

TEST_F(ScaledSphereFixture, CompareSavedAndFixtureScaledSphereMesh)
{

    EXPECT_EQ(
        _out_SphereMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedSpherePoints.size());
    EXPECT_EQ(
        _out_SphereMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedSphereCells.size());

    for (std::size_t point = 0; point < _SavedSpherePoints.size(); ++point) {

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
                EXPECT_EQ(
                    *_out_SphereMeshUsedForRegressionTestPointId,
                    _SavedSphereCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(
                    *_out_SphereMeshUsedForRegressionTestPointId,
                    _SavedSphereCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(
                    *_out_SphereMeshUsedForRegressionTestPointId,
                    _SavedSphereCells[c].v3);

            _out_SphereMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_SphereMeshUsedForRegressionTestCell;
        ++c;
    }
}

TEST_F(ScaledConeFixture, CompareSavedAndFixtureScaledConeMesh)
{

    EXPECT_EQ(
        _out_ConeMeshUsedForRegressionTest->GetNumberOfPoints(),
        _SavedConePoints.size());
    EXPECT_EQ(
        _out_ConeMeshUsedForRegressionTest->GetNumberOfCells(),
        _SavedConeCells.size());

    for (std::size_t point = 0; point < _SavedConePoints.size(); ++point) {

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
                EXPECT_EQ(
                    *_out_ConeMeshUsedForRegressionTestPointId,
                    _SavedConeCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(
                    *_out_ConeMeshUsedForRegressionTestPointId,
                    _SavedConeCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(
                    *_out_ConeMeshUsedForRegressionTestPointId,
                    _SavedConeCells[c].v3);

            _out_ConeMeshUsedForRegressionTestPointId++;
            counter++;
        }
        ++_out_ConeMeshUsedForRegressionTestCell;
        ++c;
    }
}