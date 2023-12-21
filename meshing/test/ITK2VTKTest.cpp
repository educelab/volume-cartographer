#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/shapes/Cone.hpp"
#include "vc/core/shapes/Cube.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/shapes/Sphere.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;

/*
 * The following five fixture build test inputs for the ITK2VTK tests
 *
 * Comments are included in the Plane Fixture only...the remainder of the
 * fixture copy the same ideas
 */

class ITK2VTKPlaneFixture : public ::testing::Test
{
public:
    ITK2VTKPlaneFixture()
    {
        // fill itk mesh
        _in_Mesh = _Plane.itkMesh();

        // init vtkpolydata pointers
        _out_Mesh = vtkPolyData::New();

        // call ITK2VTK
        volcart::meshing::ITK2VTK(_in_Mesh, _out_Mesh);

        // Read in the vtk.ply file
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "ITKPlaneMeshConvertedToVTK.ply", _SavedPoints, _SavedCells);
    }

    // init Plane
    volcart::shapes::Plane _Plane;

    // declare itk input mesh object
    ITKMesh::Pointer _in_Mesh;

    // declare vtk output polydata object
    vtkPolyData* _out_Mesh;

    // Declare vectors that will hold read data created by ITK2VTKExample.cpp
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class ITK2VTKCubeFixture : public ::testing::Test
{
public:
    ITK2VTKCubeFixture()
    {

        _in_Mesh = _Cube.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::ITK2VTK(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "ITKCubeMeshConvertedToVTK.ply", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Cube _Cube;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class ITK2VTKArchFixture : public ::testing::Test
{
public:
    ITK2VTKArchFixture()
    {

        _in_Mesh = _Arch.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::ITK2VTK(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "ITKArchMeshConvertedToVTK.ply", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class ITK2VTKSphereFixture : public ::testing::Test
{
public:
    ITK2VTKSphereFixture()
    {

        _in_Mesh = _Sphere.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::ITK2VTK(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "ITKSphereMeshConvertedToVTK.ply", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Sphere _Sphere;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class ITK2VTKConeFixture : public ::testing::Test
{
public:
    ITK2VTKConeFixture()
    {

        _in_Mesh = _Cone.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::ITK2VTK(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::ParsePLYFile(
            "ITKConeMeshConvertedToVTK.ply", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Cone _Cone;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

// This fixture builds a simple plane but *does not* calculate normals for the
// faces
class NoNormalsFixture : public ::testing::Test
{
public:
    NoNormalsFixture()
    {
        _itk_Mesh = ITKMesh::New();

        ITKPoint p0, p1, p2, p3;
        p0[0] = 0.0;
        p0[1] = 0.0;
        p0[2] = 0.0;
        p1[0] = 1.0;
        p1[1] = 0.0;
        p1[2] = 0.0;
        p2[0] = 0.0;
        p2[1] = 1.0;
        p2[2] = 0.0;
        p3[0] = 1.0;
        p3[1] = 1.0;
        p3[2] = 0.0;
        _itk_Mesh->SetPoint(0, p0);
        _itk_Mesh->SetPoint(1, p1);
        _itk_Mesh->SetPoint(2, p2);
        _itk_Mesh->SetPoint(3, p3);

        ITKCell::CellAutoPointer cell_0;
        ITKCell::CellAutoPointer cell_1;
        cell_0.TakeOwnership(new ITKTriangle);
        cell_1.TakeOwnership(new ITKTriangle);

        cell_0->SetPointId(0, 0);
        cell_0->SetPointId(1, 1);
        cell_0->SetPointId(2, 2);

        cell_1->SetPointId(0, 1);
        cell_1->SetPointId(1, 3);
        cell_1->SetPointId(2, 2);

        _itk_Mesh->SetCell(0, cell_0);
        _itk_Mesh->SetCell(1, cell_1);
    }

    ITKMesh::Pointer _itk_Mesh;
};

/*
 * The following five fixture build test inputs for the VTK2ITK tests
 *
 * Comments are included in the Plane Fixture only...the remainder of the fixure
 * copy the same ideas
 */

class VTK2ITKPlaneFixture : public ::testing::Test
{
public:
    VTK2ITKPlaneFixture()
    {

        _in_Mesh = _Plane.vtkMesh();
        _out_Mesh = ITKMesh::New();

        // assign smartpointer data into vtkPolyData* object
        vtkReadPlaneData = _in_Mesh.GetPointer();

        // convert from vtk mesh to itk mesh
        volcart::meshing::VTK2ITK(vtkReadPlaneData, _out_Mesh);

        // read data in from saved file created by ITK2VTKExample.cpp
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "VTKPlaneMeshConvertedToITK.obj", _SavedPoints, _SavedCells);
    }

    // init shape
    volcart::shapes::Plane _Plane;

    // declare input vtkPolyData pointer to hold input mesh data
    vtkSmartPointer<vtkPolyData> _in_Mesh;

    // pointer that will hold input mesh data for conversion to itk mesh
    vtkPolyData* vtkReadPlaneData;

    // declare mesh to hold output of VTK2ITK call
    ITKMesh::Pointer _out_Mesh;

    // declare vectors to hold points and cells from savedITK data file created
    // by ITK2VTKExample.cpp
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class VTK2ITKCubeFixture : public ::testing::Test
{
public:
    VTK2ITKCubeFixture()
    {

        _in_Mesh = _Cube.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadCubeData = _in_Mesh.GetPointer();
        volcart::meshing::VTK2ITK(vtkReadCubeData, _out_Mesh);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "VTKCubeMeshConvertedToITK.obj", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Cube _Cube;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadCubeData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class VTK2ITKArchFixture : public ::testing::Test
{
public:
    VTK2ITKArchFixture()
    {

        _in_Mesh = _Arch.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadArchData = _in_Mesh.GetPointer();
        volcart::meshing::VTK2ITK(vtkReadArchData, _out_Mesh);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "VTKArchMeshConvertedToITK.obj", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Arch _Arch;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadArchData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class VTK2ITKSphereFixture : public ::testing::Test
{
public:
    VTK2ITKSphereFixture()
    {

        _in_Mesh = _Sphere.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadSphereData = _in_Mesh.GetPointer();
        volcart::meshing::VTK2ITK(vtkReadSphereData, _out_Mesh);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "VTKSphereMeshConvertedToITK.obj", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Sphere _Sphere;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadSphereData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

class VTK2ITKConeFixture : public ::testing::Test
{
public:
    VTK2ITKConeFixture()
    {

        _in_Mesh = _Cone.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadConeData = _in_Mesh.GetPointer();
        volcart::meshing::VTK2ITK(vtkReadConeData, _out_Mesh);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "VTKConeMeshConvertedToITK.obj", _SavedPoints, _SavedCells);
    }

    volcart::shapes::Cone _Cone;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadConeData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<SimpleMesh::Vertex> _SavedPoints;
    std::vector<SimpleMesh::Cell> _SavedCells;
};

/* ITK2VTK TESTS */
/*
 * Comments included for the Plane test but omitted in subsequent tests because
 * they follow the same idea
 */

TEST_F(
    ITK2VTKPlaneFixture,
    CompareFixtureITKToVTKConvertedPlaneWithSavedPlaneVTKFileTest)
{

    // Check number of points in each mesh
    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // Now iterate over point sets and compare coordinate and normal values
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals
    vtkDataArray* out_PlanePointNormals =
        _out_Mesh->GetPointData()->GetNormals();
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(
            out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(
            out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(
            out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    // Cells

    // compare number of cells in each itk mesh
    EXPECT_EQ(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for (vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++) {

        vtkCell* out_VTKPlaneCell = _out_Mesh->GetCell(c_id);

        // Check that vertices making up the corresponding cells are equal
        // We have three checks since there are three points that make up a cell
        EXPECT_EQ(
            out_VTKPlaneCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        EXPECT_EQ(
            out_VTKPlaneCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        EXPECT_EQ(
            out_VTKPlaneCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);
    }
}

TEST_F(
    ITK2VTKCubeFixture,
    CompareFixtureITKToVTKConvertedCubeWithSavedCubeVTKFileTest)
{

    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // Now iterate over point sets and compare coordinate and normal values
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals
    vtkDataArray* out_PlanePointNormals =
        _out_Mesh->GetPointData()->GetNormals();
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(
            out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(
            out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(
            out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    EXPECT_EQ(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for (int c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++) {

        vtkCell* out_VTKCubeCell = _out_Mesh->GetCell(c_id);

        EXPECT_EQ(
            out_VTKCubeCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        EXPECT_EQ(
            out_VTKCubeCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        EXPECT_EQ(
            out_VTKCubeCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);
    }
}

TEST_F(
    ITK2VTKArchFixture,
    CompareFixtureITKToVTKConvertedArchWithSavedArchVTKFileTest)
{

    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // Now iterate over point sets and compare coordinate and normal values
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals
    vtkDataArray* out_PlanePointNormals =
        _out_Mesh->GetPointData()->GetNormals();
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(
            out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(
            out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(
            out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    EXPECT_EQ(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for (vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++) {

        vtkCell* out_VTKArchCell = _out_Mesh->GetCell(c_id);

        EXPECT_EQ(
            out_VTKArchCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        EXPECT_EQ(
            out_VTKArchCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        EXPECT_EQ(
            out_VTKArchCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);
    }
}

TEST_F(
    ITK2VTKSphereFixture,
    CompareFixtureITKToVTKConvertedSphereWithSavedSphereVTKFileTest)
{

    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // Now iterate over point sets and compare coordinate and normal values
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals
    vtkDataArray* out_PlanePointNormals =
        _out_Mesh->GetPointData()->GetNormals();
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(
            out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(
            out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(
            out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    EXPECT_EQ(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for (vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++) {

        vtkCell* out_VTKSphereCell = _out_Mesh->GetCell(c_id);

        EXPECT_EQ(
            out_VTKSphereCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        EXPECT_EQ(
            out_VTKSphereCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        EXPECT_EQ(
            out_VTKSphereCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);
    }
}

TEST_F(
    ITK2VTKConeFixture,
    CompareFixtureITKToVTKConvertedConeWithSavedConeVTKFileTest)
{

    EXPECT_EQ(_out_Mesh->GetNumberOfPoints(), _SavedPoints.size());

    // Now iterate over point sets and compare coordinate and normal values
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals
    vtkDataArray* out_PlanePointNormals =
        _out_Mesh->GetPointData()->GetNormals();
    for (vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints();
         ++pnt_id) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(
            out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(
            out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(
            out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    EXPECT_EQ(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for (vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++) {

        vtkCell* out_VTKConeCell = _out_Mesh->GetCell(c_id);

        EXPECT_EQ(
            out_VTKConeCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        EXPECT_EQ(
            out_VTKConeCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        EXPECT_EQ(
            out_VTKConeCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);
    }
}

/* VTK2ITK TESTS */
/*
 * Comments included for the Plane test but omitted in subsequent tests because
 * they follow the same idea
 */

TEST_F(
    VTK2ITKPlaneFixture,
    CompareFixtureVTKToITKConvertedPlaneWithSavedPlaneITKFileTest)
{

    // Confirm equal number of points in both the saved itk mesh and the output
    // itk mesh from the fixture
    EXPECT_EQ(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    // Check equivalency of points
    for (size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for (int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    // Cells (faces)

    // Initialize Cell Iterators
    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();

    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        // Initialize Iterators for Points in a Cell
        ITKPointInCellIterator _out_MeshPointId =
            _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        // while we have points in the cell
        while (_out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd()) {

            // Now to check the points within the cells
            if (counter == 0)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v3);

            // increment points
            _out_MeshPointId++;
            counter++;
        }

        // increment cells
        ++_out_MeshCell;
        ++c;
    }
}

TEST_F(
    VTK2ITKCubeFixture,
    CompareFixtureVTKToITKConvertedCubeWithSavedCubeITKFileTest)
{

    EXPECT_EQ(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    // Check equivalency of points
    for (size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for (int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId =
            _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while (_out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;
        }

        ++_out_MeshCell;
        ++c;
    }
}

TEST_F(
    VTK2ITKArchFixture,
    CompareFixtureVTKToITKConvertedArchWithSavedArchITKFileTest)
{

    EXPECT_EQ(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    // Check equivalency of points
    for (size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for (int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId =
            _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while (_out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;
        }

        ++_out_MeshCell;
        ++c;
    }
}

TEST_F(
    VTK2ITKSphereFixture,
    CompareFixtureVTKToITKConvertedSphereWithSavedSphereITKFileTest)
{

    EXPECT_EQ(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    // Check equivalency of points
    for (size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for (int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId =
            _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while (_out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;
        }

        ++_out_MeshCell;
        ++c;
    }
}

TEST_F(
    VTK2ITKConeFixture,
    CompareFixtureVTKToITKConvertedConeWithSavedConeITKFileTest)
{

    EXPECT_EQ(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    // Check equivalency of points
    for (size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(
            _out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for (int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId =
            _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while (_out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd()) {

            if (counter == 0)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v1);
            else if (counter == 1)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                EXPECT_EQ(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;
        }

        ++_out_MeshCell;
        ++c;
    }
}

/* EDGE CASE TESTS */
// Test whether things fail if the meshes don't have normals
TEST_F(NoNormalsFixture, MeshWithNoNormals)
{
    // ITK to VTK
    auto vtk_Mesh = vtkSmartPointer<vtkPolyData>::New();
    volcart::meshing::ITK2VTK(_itk_Mesh, vtk_Mesh);

    // VTK to ITK
    ITKMesh::Pointer itk_Mesh2 = ITKMesh::New();
    volcart::meshing::VTK2ITK(vtk_Mesh, itk_Mesh2);
}