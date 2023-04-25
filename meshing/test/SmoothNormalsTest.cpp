#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Cone.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Sphere.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/meshing/SmoothNormals.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;

/*
 * This builds objects for the case below that reference
 * the fixture as their second argument
 *
 */

class SmoothNormalsFixture : public ::testing::Test
{
public:
    SmoothNormalsFixture()
    {

        // smoothing radius and _Tolerance value for later comparisons
        _SmoothingFactor = 2;

        // assign input meshes that will be smoothed
        _in_PlaneMesh = _Plane.itkMesh();
        _in_CubeMesh = _Cube.itkMesh();
        _in_ArchMesh = _Arch.itkMesh();
        _in_SphereMesh = _Sphere.itkMesh();
        _in_ConeMesh = _Cone.itkMesh();

        // call SmoothNormals and assign results to output meshes
        _out_SmoothedPlaneMesh =
            volcart::meshing::SmoothNormals(_in_PlaneMesh, _SmoothingFactor);
        _out_SmoothedCubeMesh =
            volcart::meshing::SmoothNormals(_in_CubeMesh, _SmoothingFactor);
        _out_SmoothedArchMesh =
            volcart::meshing::SmoothNormals(_in_ArchMesh, _SmoothingFactor);
        _out_SmoothedSphereMesh =
            volcart::meshing::SmoothNormals(_in_SphereMesh, _SmoothingFactor);
        _out_SmoothedConeMesh =
            volcart::meshing::SmoothNormals(_in_ConeMesh, _SmoothingFactor);

        // read in saved obj files created by SmoothNormalsExample.cpp
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "PlaneWithSmoothedNormals.obj", _SavedPlanePoints,
            _SavedPlaneCells);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "CubeWithSmoothedNormals.obj", _SavedCubePoints, _SavedCubeCells);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "ArchWithSmoothedNormals.obj", _SavedArchPoints, _SavedArchCells);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "SphereWithSmoothedNormals.obj", _SavedSpherePoints,
            _SavedSphereCells);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "ConeWithSmoothedNormals.obj", _SavedConePoints, _SavedConeCells);
    }

    // init input and output mesh ptrs
    ITKMesh::Pointer _in_PlaneMesh, _in_CubeMesh, _in_SphereMesh, _in_ArchMesh,
        _in_ConeMesh;
    ITKMesh::Pointer _out_SmoothedPlaneMesh, _out_SmoothedCubeMesh,
        _out_SmoothedSphereMesh, _out_SmoothedArchMesh, _out_SmoothedConeMesh;
    // init shapes
    volcart::shapes::Plane _Plane;
    volcart::shapes::Cube _Cube;
    volcart::shapes::Sphere _Sphere;
    volcart::shapes::Arch _Arch;
    volcart::shapes::Cone _Cone;

    double _SmoothingFactor;

    // init vectors to hold points and cells from savedITK data files
    std::vector<SimpleMesh::Vertex> _SavedPlanePoints, _SavedCubePoints,
        _SavedArchPoints, _SavedSpherePoints, _SavedConePoints;
    std::vector<SimpleMesh::Cell> _SavedPlaneCells, _SavedCubeCells,
        _SavedArchCells, _SavedSphereCells, _SavedConeCells;
};

/*
 * The next four tests use the obj files representing the smoothed shapes
 * created by smoothingExample.cpp
 * and compares these files to test-specific calls SmoothNormals() using the
 * same shape objects.
 *
 * Smoothing factor should be 2 for each test case
 *
 * Split the tests into four cases for log purposes and pinpointing errors
 * faster if there should be
 * an issue in the future.
 *
 */

TEST_F(SmoothNormalsFixture, CompareFixtureSmoothedPlaneWithSavedPlaneTest)
{

    // Check number of points in each mesh
    EXPECT_EQ(
        _out_SmoothedPlaneMesh->GetNumberOfPoints(), _SavedPlanePoints.size());
    EXPECT_EQ(
        _out_SmoothedPlaneMesh->GetNumberOfCells(), _SavedPlaneCells.size());

    // points
    for (size_t p_id = 0; p_id < _out_SmoothedPlaneMesh->GetNumberOfPoints();
         ++p_id) {
        volcart::testing::SmallOrClose(
            _out_SmoothedPlaneMesh->GetPoint(p_id)[0],
            _SavedPlanePoints[p_id].x);
        volcart::testing::SmallOrClose(
            _out_SmoothedPlaneMesh->GetPoint(p_id)[1],
            _SavedPlanePoints[p_id].y);
        volcart::testing::SmallOrClose(
            _out_SmoothedPlaneMesh->GetPoint(p_id)[2],
            _SavedPlanePoints[p_id].z);
    }

    // normals
    ITKPointIterator point = _out_SmoothedPlaneMesh->GetPoints()->Begin();
    for (int p = 0; point != _out_SmoothedPlaneMesh->GetPoints()->End();
         ++p, ++point) {
        ITKPixel out_Normal;
        _out_SmoothedPlaneMesh->GetPointData(point.Index(), &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedPlanePoints[p].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedPlanePoints[p].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedPlanePoints[p].nz);
    }

    // cells
    ITKCellIterator out_PlaneCell = _out_SmoothedPlaneMesh->GetCells()->Begin();

    int c = 0;

    while (out_PlaneCell != _out_SmoothedPlaneMesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator out_PlaneMeshPointId =
            out_PlaneCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (out_PlaneMeshPointId != out_PlaneCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                EXPECT_EQ(*out_PlaneMeshPointId, _SavedPlaneCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*out_PlaneMeshPointId, _SavedPlaneCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*out_PlaneMeshPointId, _SavedPlaneCells[c].v3);

            // increment points
            out_PlaneMeshPointId++;
            counter++;
        }

        // increment cells
        ++out_PlaneCell;
        ++c;
    }
}

//           //
//           //
//   CUBE    //
//           //
//           //

TEST_F(SmoothNormalsFixture, CompareFixtureSmoothedCubeWithSavedCubeTest)
{

    // Check number of points and cells in each mesh
    EXPECT_EQ(
        _out_SmoothedCubeMesh->GetNumberOfPoints(), _SavedCubePoints.size());
    EXPECT_EQ(
        _out_SmoothedCubeMesh->GetNumberOfCells(), _SavedCubeCells.size());

    // points
    for (size_t p_id = 0; p_id < _out_SmoothedCubeMesh->GetNumberOfPoints();
         ++p_id) {
        volcart::testing::SmallOrClose(
            _out_SmoothedCubeMesh->GetPoint(p_id)[0], _SavedCubePoints[p_id].x);
        volcart::testing::SmallOrClose(
            _out_SmoothedCubeMesh->GetPoint(p_id)[1], _SavedCubePoints[p_id].y);
        volcart::testing::SmallOrClose(
            _out_SmoothedCubeMesh->GetPoint(p_id)[2], _SavedCubePoints[p_id].z);
    }

    // normals
    ITKPointIterator point = _out_SmoothedCubeMesh->GetPoints()->Begin();
    for (int p = 0; point != _out_SmoothedCubeMesh->GetPoints()->End();
         ++p, ++point) {
        ITKPixel out_Normal;
        _out_SmoothedCubeMesh->GetPointData(point.Index(), &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedCubePoints[p].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedCubePoints[p].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedCubePoints[p].nz);
    }

    // cells
    ITKCellIterator out_CubeCell = _out_SmoothedCubeMesh->GetCells()->Begin();

    int c = 0;

    while (out_CubeCell != _out_SmoothedCubeMesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator out_CubeMeshPointId =
            out_CubeCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (out_CubeMeshPointId != out_CubeCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                EXPECT_EQ(*out_CubeMeshPointId, _SavedCubeCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*out_CubeMeshPointId, _SavedCubeCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*out_CubeMeshPointId, _SavedCubeCells[c].v3);

            // increment points
            out_CubeMeshPointId++;
            counter++;
        }

        // increment cells
        ++out_CubeCell;
        ++c;
    }
}

//             //
//             //
//   SPHERE    //
//             //
//             //

TEST_F(SmoothNormalsFixture, CompareFixtureSmoothedSphereWithSavedSphereTest)
{

    // Check number of points in each mesh
    EXPECT_EQ(
        _out_SmoothedSphereMesh->GetNumberOfPoints(),
        _SavedSpherePoints.size());
    EXPECT_EQ(
        _out_SmoothedSphereMesh->GetNumberOfCells(), _SavedSphereCells.size());

    // points
    for (size_t p_id = 0; p_id < _out_SmoothedSphereMesh->GetNumberOfPoints();
         ++p_id) {
        volcart::testing::SmallOrClose(
            _out_SmoothedSphereMesh->GetPoint(p_id)[0],
            _SavedSpherePoints[p_id].x);
        volcart::testing::SmallOrClose(
            _out_SmoothedSphereMesh->GetPoint(p_id)[1],
            _SavedSpherePoints[p_id].y);
        volcart::testing::SmallOrClose(
            _out_SmoothedSphereMesh->GetPoint(p_id)[2],
            _SavedSpherePoints[p_id].z);
    }

    // normals
    ITKPointIterator point = _out_SmoothedSphereMesh->GetPoints()->Begin();
    for (int p = 0; point != _out_SmoothedSphereMesh->GetPoints()->End();
         ++p, ++point) {
        ITKPixel out_Normal;
        _out_SmoothedSphereMesh->GetPointData(point.Index(), &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedSpherePoints[p].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedSpherePoints[p].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedSpherePoints[p].nz);
    }

    // cells
    ITKCellIterator out_SphereCell =
        _out_SmoothedSphereMesh->GetCells()->Begin();

    int c = 0;

    while (out_SphereCell != _out_SmoothedSphereMesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator out_SphereMeshPointId =
            out_SphereCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (out_SphereMeshPointId != out_SphereCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                EXPECT_EQ(*out_SphereMeshPointId, _SavedSphereCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*out_SphereMeshPointId, _SavedSphereCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*out_SphereMeshPointId, _SavedSphereCells[c].v3);

            // increment points
            out_SphereMeshPointId++;
            counter++;
        }

        // increment cells
        ++out_SphereCell;
        ++c;
    }
}

//           //
//   ARCH    //
//           //

TEST_F(SmoothNormalsFixture, CompareFixtureSmoothedArchWithSavedArchTest)
{

    // Check number of points in each mesh
    EXPECT_EQ(
        _out_SmoothedArchMesh->GetNumberOfPoints(), _SavedArchPoints.size());
    EXPECT_EQ(
        _out_SmoothedArchMesh->GetNumberOfCells(), _SavedArchCells.size());

    // points
    for (size_t p_id = 0; p_id < _out_SmoothedArchMesh->GetNumberOfPoints();
         ++p_id) {
        volcart::testing::SmallOrClose(
            _out_SmoothedArchMesh->GetPoint(p_id)[0], _SavedArchPoints[p_id].x);
        volcart::testing::SmallOrClose(
            _out_SmoothedArchMesh->GetPoint(p_id)[1], _SavedArchPoints[p_id].y);
        volcart::testing::SmallOrClose(
            _out_SmoothedArchMesh->GetPoint(p_id)[2], _SavedArchPoints[p_id].z);
    }

    // normals
    ITKPointIterator point = _out_SmoothedArchMesh->GetPoints()->Begin();
    for (int p = 0; point != _out_SmoothedArchMesh->GetPoints()->End();
         ++p, ++point) {
        ITKPixel out_Normal;
        _out_SmoothedArchMesh->GetPointData(point.Index(), &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedArchPoints[p].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedArchPoints[p].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedArchPoints[p].nz);
    }

    // Initialize Cell Iterators
    ITKCellIterator out_ArchCell = _out_SmoothedArchMesh->GetCells()->Begin();

    int c = 0;

    while (out_ArchCell != _out_SmoothedArchMesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator out_ArchMeshPointId =
            out_ArchCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (out_ArchMeshPointId != out_ArchCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                EXPECT_EQ(*out_ArchMeshPointId, _SavedArchCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*out_ArchMeshPointId, _SavedArchCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*out_ArchMeshPointId, _SavedArchCells[c].v3);

            // increment points
            out_ArchMeshPointId++;
            counter++;
        }

        // increment cells
        ++out_ArchCell;
        ++c;
    }
}

//           //
//   CONE    //
//           //

TEST_F(SmoothNormalsFixture, CompareFixtureSmoothedConeWithSavedConeTest)
{

    // Check number of points in each mesh
    EXPECT_EQ(
        _out_SmoothedConeMesh->GetNumberOfPoints(), _SavedConePoints.size());

    // points
    for (size_t p_id = 0; p_id < _out_SmoothedConeMesh->GetNumberOfPoints();
         ++p_id) {
        volcart::testing::SmallOrClose(
            _out_SmoothedConeMesh->GetPoint(p_id)[0], _SavedConePoints[p_id].x);
        volcart::testing::SmallOrClose(
            _out_SmoothedConeMesh->GetPoint(p_id)[1], _SavedConePoints[p_id].y);
        volcart::testing::SmallOrClose(
            _out_SmoothedConeMesh->GetPoint(p_id)[2], _SavedConePoints[p_id].z);
    }

    // normals
    ITKPointIterator point = _out_SmoothedConeMesh->GetPoints()->Begin();
    for (int p = 0; point != _out_SmoothedConeMesh->GetPoints()->End();
         ++p, ++point) {
        ITKPixel out_Normal;
        _out_SmoothedConeMesh->GetPointData(point.Index(), &out_Normal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(out_Normal[0], _SavedConePoints[p].nx);
        volcart::testing::SmallOrClose(out_Normal[1], _SavedConePoints[p].ny);
        volcart::testing::SmallOrClose(out_Normal[2], _SavedConePoints[p].nz);
    }

    //               //
    // compare cells //
    //               //

    EXPECT_EQ(
        _out_SmoothedConeMesh->GetNumberOfCells(), _SavedConeCells.size());

    // Initialize Cell Iterators
    ITKCellIterator out_ConeCell = _out_SmoothedConeMesh->GetCells()->Begin();

    int c_id = 0;

    while (out_ConeCell != _out_SmoothedConeMesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator out_ConeMeshPointId =
            out_ConeCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (out_ConeMeshPointId != out_ConeCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                EXPECT_EQ(*out_ConeMeshPointId, _SavedConeCells[c_id].v1);
            else if (counter == 1)
                EXPECT_EQ(*out_ConeMeshPointId, _SavedConeCells[c_id].v2);
            else if (counter == 2)
                EXPECT_EQ(*out_ConeMeshPointId, _SavedConeCells[c_id].v3);

            // increment points
            out_ConeMeshPointId++;
            counter++;
        }

        // increment cells
        ++out_ConeCell;
        ++c_id;
    }
}

/*
 * Testing a zero radius smoothing factor
 *
 * Expected output would be an unchanged mesh
 *
 */

TEST_F(SmoothNormalsFixture, SmoothWithZeroRadiusTest)
{

    // call SmoothNormals() and assign results
    ITKMesh::Pointer ZeroRadiusSmoothedMesh =
        volcart::meshing::SmoothNormals(_in_ArchMesh, 0);

    // check number of points and cells are equivalent between the two meshes
    EXPECT_EQ(
        _in_ArchMesh->GetNumberOfPoints(),
        ZeroRadiusSmoothedMesh->GetNumberOfPoints());
    EXPECT_EQ(
        _in_ArchMesh->GetNumberOfCells(),
        ZeroRadiusSmoothedMesh->GetNumberOfCells());

    // compare point values
    for (size_t p_id = 0; p_id < _in_ArchMesh->GetNumberOfPoints(); ++p_id) {
        volcart::testing::SmallOrClose(
            _in_ArchMesh->GetPoint(p_id)[0],
            ZeroRadiusSmoothedMesh->GetPoint(p_id)[0]);
        volcart::testing::SmallOrClose(
            _in_ArchMesh->GetPoint(p_id)[1],
            ZeroRadiusSmoothedMesh->GetPoint(p_id)[1]);
        volcart::testing::SmallOrClose(
            _in_ArchMesh->GetPoint(p_id)[2],
            ZeroRadiusSmoothedMesh->GetPoint(p_id)[2]);
    }

    // compare normals
    for (auto point = _in_ArchMesh->GetPoints()->Begin();
         point != _in_ArchMesh->GetPoints()->End(); ++point) {

        ITKPixel in_ArchNormal, ZeroRadiusNormal;
        _in_ArchMesh->GetPointData(point.Index(), &in_ArchNormal);
        ZeroRadiusSmoothedMesh->GetPointData(point.Index(), &ZeroRadiusNormal);

        // Now compare the normals for the two meshes
        volcart::testing::SmallOrClose(in_ArchNormal[0], ZeroRadiusNormal[0]);
        volcart::testing::SmallOrClose(in_ArchNormal[1], ZeroRadiusNormal[1]);
        volcart::testing::SmallOrClose(in_ArchNormal[2], ZeroRadiusNormal[2]);
    }

    //               //
    // compare cells //
    //               //

    // Initialize Cell Iterators
    ITKCellIterator in_ArchCell = _in_ArchMesh->GetCells()->Begin();
    ITKCellIterator ZeroRadiusSmoothedCell =
        ZeroRadiusSmoothedMesh->GetCells()->Begin();

    while (in_ArchCell != _in_ArchMesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator in_ArchMeshPointId =
            in_ArchCell.Value()->PointIdsBegin();
        ITKPointInCellIterator ZeroRadiusMeshPointId =
            ZeroRadiusSmoothedCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (in_ArchMeshPointId != in_ArchCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                EXPECT_EQ(*in_ArchMeshPointId, *ZeroRadiusMeshPointId);
            else if (counter == 1)
                EXPECT_EQ(*in_ArchMeshPointId, *ZeroRadiusMeshPointId);
            else if (counter == 2)
                EXPECT_EQ(*in_ArchMeshPointId, *ZeroRadiusMeshPointId);

            // increment points
            ++in_ArchMeshPointId;
            ++ZeroRadiusMeshPointId;
            ++counter;
        }

        // increment cells
        ++in_ArchCell;
        ++ZeroRadiusSmoothedCell;
    }
}