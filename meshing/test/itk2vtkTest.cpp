//
// Created by Ryan Taber on 10/14/15.
//

#define BOOST_TEST_MODULE itk2vtk

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>

#include "common/vc_defines.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Arch.h"
#include "common/shapes/Sphere.h"
#include "common/shapes/Cone.h"
#include "meshing/itk2vtk.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"

using namespace volcart;

/************************************************************************************
 *                                                                                  *
 *  it2vtkTest.cpp - tests the functionality of /v-c/meshing/it2vtk.cpp             *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether an itk mesh can be converted to a vtk mesh               *
 *           and vice versa.                                                        *
 *                                                                                  *
 *        2. confirm saved .obj and .ply files match test-generated obj and         *
 *           ply files after running itk2vtk and vtk2itk                            *
 *                                                                                  *
 *  This file is broken up into two test fixtures (ivFix & viFix) which initialize  *
 *  the objects used in each of the four test cases.                                *
 *                                                                                  *
 *  1. i2v (fixture test case):                                                     *
 *                                                                                  *
 *      Takes an itk mesh created from fixture and data to vtkpolydata pointer.     *
 *      Vtkpolydata then converted back to itk mesh. Successful test if converted   *
 *      itk matches original itk mesh data points                                   *
 *                                                                                  *
 *  2. v2i (fixture test case):                                                     *
 *                                                                                  *
 *      Same idea as i2v test case except that the test starts with vtkpolydata.    *
 *      The vtk data is converted into itk mesh and back to vtk. Success if the     *
 *      original vtk and converted vtk data points match.                           *
 *                                                                                  *
 *   3. compareSavedITK (fixture test case):                                        *
 *      Read in itk.obj and compare with mesh created from vtk2itk call result.     *
 *                                                                                  *
 *   4. compreSavedVTK (fixture test case):                                         *
 *      Read in vtk.ply and compare with mesh created from itk2vtk call result.     *
 *                                                                                  *
 * Input:                                                                           *
 *     No required inputs for this sample test. All test objects are created        *
 *     internally.                                                                  *
 *                                                                                  *
 * Test-Specific Output:                                                            *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output.                                          *
 *                                                                                  *
 * Miscellaneous:                                                                   *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/

/*
 * The following five fixture build test inputs for the itk2vtk tests
 *
 * Comments are included in the Plane Fixture only...the remainder of the fixure copy the same ideas
 */

struct itk2vtkPlaneFixture {

    itk2vtkPlaneFixture() {

        //fill itk mesh
        _in_Mesh = _Plane.itkMesh();

        //init vtkpolydata pointers
        _out_Mesh = vtkPolyData::New();

        //call itk2vtk
        volcart::meshing::itk2vtk(_in_Mesh, _out_Mesh);

        //Read in the vtk.ply file
        volcart::testing::ParsingHelpers::parsePlyFile("ITKPlaneMeshConvertedToVTK.ply",
                                                                            _SavedPoints, _SavedCells);

        //std::cerr << "setting up itk2vtk Plane test objects" << std::endl;
    }

    ~itk2vtkPlaneFixture(){ /*std::cerr << "cleaning up itk2vtk Plane test objects" << std::endl;*/ }

    //init Plane
    volcart::shapes::Plane _Plane;

    //declare itk input mesh object
    ITKMesh::Pointer _in_Mesh;

    //declare vtk output polydata object
    vtkPolyData* _out_Mesh;

    //Declare vectors that will hold read data created by itk2vtkExample.cpp
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;
};

struct itk2vtkCubeFixture {

    itk2vtkCubeFixture() {

        _in_Mesh = _Cube.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKCubeMeshConvertedToVTK.ply",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up itk2vtk Cube test objects" << std::endl;
    }

    ~itk2vtkCubeFixture(){ /*std::cerr << "cleaning up itk2vtk Cube test objects" << std::endl;*/ }

    volcart::shapes::Cube _Cube;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};


struct itk2vtkArchFixture {

    itk2vtkArchFixture() {

        _in_Mesh = _Arch.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKArchMeshConvertedToVTK.ply",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up itk2vtk Arch test objects" << std::endl;
    }

    ~itk2vtkArchFixture(){ /*std::cerr << "cleaning up itk2vtk Arch test objects" << std::endl;*/ }

    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};

struct itk2vtkSphereFixture {

    itk2vtkSphereFixture() {

        _in_Mesh = _Sphere.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKSphereMeshConvertedToVTK.ply",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up itk2vtk Sphere test objects" << std::endl;
    }

    ~itk2vtkSphereFixture(){ /*std::cerr << "cleaning up itk2vtk Sphere test objects" << std::endl;*/ }

    volcart::shapes::Sphere _Sphere;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};

struct itk2vtkConeFixture {

    itk2vtkConeFixture() {

        _in_Mesh = _Cone.itkMesh();
        _out_Mesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_Mesh, _out_Mesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKConeMeshConvertedToVTK.ply",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up itk2vtk Cone test objects" << std::endl;
    }

    ~itk2vtkConeFixture(){ /*std::cerr << "cleaning up itk2vtk Cone test objects" << std::endl;*/ }

    volcart::shapes::Cone _Cone;
    ITKMesh::Pointer _in_Mesh;
    vtkPolyData* _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};

// This fixture builds a simple plane but *does not* calculate normals for the faces
struct NoNormalsFixture {
    NoNormalsFixture(){
        _itk_Mesh = ITKMesh::New();

        ITKPoint p0, p1, p2, p3;
        p0[0] = 0.0; p0[1] = 0.0; p0[2] = 0.0;
        p1[0] = 1.0; p1[1] = 0.0; p1[2] = 0.0;
        p2[0] = 0.0; p2[1] = 1.0; p2[2] = 0.0;
        p3[0] = 1.0; p3[1] = 1.0; p3[2] = 0.0;
        _itk_Mesh->SetPoint(0, p0);
        _itk_Mesh->SetPoint(1, p1);
        _itk_Mesh->SetPoint(2, p2);
        _itk_Mesh->SetPoint(3, p3);

        ITKCell::CellAutoPointer cell_0;
        ITKCell::CellAutoPointer cell_1;
        cell_0.TakeOwnership(new ITKTriangle);
        cell_1.TakeOwnership(new ITKTriangle);

        cell_0->SetPointId(0,0);
        cell_0->SetPointId(1,1);
        cell_0->SetPointId(2,2);

        cell_1->SetPointId(0,1);
        cell_1->SetPointId(1,3);
        cell_1->SetPointId(2,2);

        _itk_Mesh->SetCell(0,cell_0);
        _itk_Mesh->SetCell(1,cell_1);
    }

    ITKMesh::Pointer _itk_Mesh;
};


/*
 * The following five fixture build test inputs for the vtk2itk tests
 *
 * Comments are included in the Plane Fixture only...the remainder of the fixure copy the same ideas
 */

struct vtk2itkPlaneFixture {

    vtk2itkPlaneFixture() {

        _in_Mesh = _Plane.vtkMesh();
        _out_Mesh = ITKMesh::New();

        // assign smartpointer data into vtkPolyData* object
        vtkReadPlaneData = _in_Mesh.GetPointer();

        // convert from vtk mesh to itk mesh
        volcart::meshing::vtk2itk(vtkReadPlaneData, _out_Mesh);

        // read data in from saved file created by itk2vtkExample.cpp
        volcart::testing::ParsingHelpers::parseObjFile("VTKPlaneMeshConvertedToITK.obj",
                                                                           _SavedPoints, _SavedCells);

        //std::cerr << "setting up vtk2itk Plane test objects" << std::endl;
    }

    ~vtk2itkPlaneFixture(){ /*std::cerr << "cleaning up vtk2itk Plane test objects" << std::endl;*/ }

    //init shape
    volcart::shapes::Plane _Plane;

    //declare input vtkPolyData pointer to hold input mesh data
    vtkSmartPointer<vtkPolyData> _in_Mesh;

    //pointer that will hold input mesh data for conversion to itk mesh
    vtkPolyData* vtkReadPlaneData;

    //declare mesh to hold output of vtk2itk call
    ITKMesh::Pointer _out_Mesh;

    //declare vectors to hold points and cells from savedITK data file created by itk2vtkExample.cpp
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};

struct vtk2itkCubeFixture {

    vtk2itkCubeFixture() {

        _in_Mesh = _Cube.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadCubeData = _in_Mesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadCubeData, _out_Mesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKCubeMeshConvertedToITK.obj",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up vtk2itk Cube test objects" << std::endl;
    }

    ~vtk2itkCubeFixture(){ /*std::cerr << "cleaning up vtk2itk Cube test objects" << std::endl;*/ }

    volcart::shapes::Cube _Cube;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadCubeData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};


struct vtk2itkArchFixture {

    vtk2itkArchFixture() {

        _in_Mesh = _Arch.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadArchData = _in_Mesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadArchData, _out_Mesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKArchMeshConvertedToITK.obj",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up vtk2itk Arch test objects" << std::endl;
    }

    ~vtk2itkArchFixture(){ /*std::cerr << "cleaning up vtk2itk Arch test objects" << std::endl;*/ }

    volcart::shapes::Arch _Arch;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadArchData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};

struct vtk2itkSphereFixture {

    vtk2itkSphereFixture() {

        _in_Mesh = _Sphere.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadSphereData = _in_Mesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadSphereData, _out_Mesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKSphereMeshConvertedToITK.obj",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up vtk2itk Sphere test objects" << std::endl;
    }

    ~vtk2itkSphereFixture(){ /*std::cerr << "cleaning up vtk2itk Sphere test objects" << std::endl*/ }

    volcart::shapes::Sphere _Sphere;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadSphereData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};

struct vtk2itkConeFixture {

    vtk2itkConeFixture() {

        _in_Mesh = _Cone.vtkMesh();
        _out_Mesh = ITKMesh::New();
        vtkReadConeData = _in_Mesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadConeData, _out_Mesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKConeMeshConvertedToITK.obj",
                                                       _SavedPoints, _SavedCells);

        //std::cerr << "setting up vtk2itk Cone test objects" << std::endl;
    }

    ~vtk2itkConeFixture(){ /*std::cerr << "cleaning up vtk2itk Cone test objects" << std::endl;*/ }

    volcart::shapes::Cone _Cone;
    vtkSmartPointer<vtkPolyData> _in_Mesh;
    vtkPolyData* vtkReadConeData;
    ITKMesh::Pointer _out_Mesh;
    std::vector<Vertex> _SavedPoints;
    std::vector<Cell> _SavedCells;

};



/**********************************************************************************************************************/
/**********************************************************************************************************************/
/*                                                                                                                    */
/*                                                  ITK2VTK TESTS                                                     */
/*                                                                                                                    */
/**********************************************************************************************************************/
/**********************************************************************************************************************/

/*
 * Comments included for the Plane test but omitted in subsequent tests because they follow the same idea
 */

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedPlaneWithSavedPlaneVTKFileTest, itk2vtkPlaneFixture){

    //Check number of points in each mesh
    BOOST_CHECK_EQUAL( _out_Mesh->GetNumberOfPoints(), _SavedPoints.size() );

    //Now iterate over point sets and compare coordinate and normal values
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    //Normals
    vtkDataArray *out_PlanePointNormals = _out_Mesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id ) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    //Cells

    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKPlaneCell = _out_Mesh->GetCell(c_id);

        //Check that vertices making up the corresponding cells are equal
        //We have three checks since there are three points that make up a cell
        BOOST_CHECK_EQUAL(out_VTKPlaneCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKPlaneCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKPlaneCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);

    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedCubeWithSavedCubeVTKFileTest, itk2vtkCubeFixture){

    BOOST_CHECK_EQUAL( _out_Mesh->GetNumberOfPoints(), _SavedPoints.size() );

    //Now iterate over point sets and compare coordinate and normal values
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    //Normals
    vtkDataArray *out_PlanePointNormals = _out_Mesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id ) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for ( int c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKCubeCell = _out_Mesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKCubeCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKCubeCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKCubeCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);

    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedArchWithSavedArchVTKFileTest, itk2vtkArchFixture){

    BOOST_CHECK_EQUAL( _out_Mesh->GetNumberOfPoints(), _SavedPoints.size() );

    //Now iterate over point sets and compare coordinate and normal values
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    //Normals
    vtkDataArray *out_PlanePointNormals = _out_Mesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id ) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKArchCell = _out_Mesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKArchCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKArchCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKArchCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);

    }
}



BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedSphereWithSavedSphereVTKFileTest, itk2vtkSphereFixture){

    BOOST_CHECK_EQUAL( _out_Mesh->GetNumberOfPoints(), _SavedPoints.size() );

    //Now iterate over point sets and compare coordinate and normal values
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    //Normals
    vtkDataArray *out_PlanePointNormals = _out_Mesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id ) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKSphereCell = _out_Mesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKSphereCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKSphereCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKSphereCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);

    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedConeWithSavedConeVTKFileTest, itk2vtkConeFixture){

    BOOST_CHECK_EQUAL( _out_Mesh->GetNumberOfPoints(), _SavedPoints.size() );

    //Now iterate over point sets and compare coordinate and normal values
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    //Normals
    vtkDataArray *out_PlanePointNormals = _out_Mesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id ) {

        ITKPixel out_NormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        volcart::testing::SmallOrClose(out_NormalTuple[0], _SavedPoints[pnt_id].nx);
        volcart::testing::SmallOrClose(out_NormalTuple[1], _SavedPoints[pnt_id].ny);
        volcart::testing::SmallOrClose(out_NormalTuple[2], _SavedPoints[pnt_id].nz);
    }

    BOOST_CHECK_EQUAL(_out_Mesh->GetNumberOfCells(), _SavedCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_Mesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKConeCell = _out_Mesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKConeCell->GetPointIds()->GetId(0), _SavedCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKConeCell->GetPointIds()->GetId(1), _SavedCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKConeCell->GetPointIds()->GetId(2), _SavedCells[c_id].v3);

    }
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/*                                                                                                                    */
/*                                                  VTK2ITK TESTS                                                     */
/*                                                                                                                    */
/**********************************************************************************************************************/
/**********************************************************************************************************************/

/*
 * Comments included for the Plane test but omitted in subsequent tests because they follow the same idea
 */

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedPlaneWithSavedPlaneITKFileTest, vtk2itkPlaneFixture){


    //Confirm equal number of points in both the saved itk mesh and the output itk mesh from the fixture
    BOOST_CHECK_EQUAL(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    //Check equivalency of points
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for ( int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point ) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    //Cells (faces)

    // Initialize Cell Iterators
    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();

    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        ITKPointInCellIterator _out_MeshPointId = _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( _out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v3);

            //increment points
            _out_MeshPointId++;
            counter++;

        }

        //increment cells
        ++_out_MeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedCubeWithSavedCubeITKFileTest, vtk2itkCubeFixture){

    BOOST_CHECK_EQUAL(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    //Check equivalency of points
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for ( int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point ) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId = _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd() ) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;
        }

        ++_out_MeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedArchWithSavedArchITKFileTest, vtk2itkArchFixture){

    BOOST_CHECK_EQUAL(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    //Check equivalency of points
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for ( int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point ) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId = _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd() ) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;

        }

        ++_out_MeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedSphereWithSavedSphereITKFileTest, vtk2itkSphereFixture){

    BOOST_CHECK_EQUAL(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    //Check equivalency of points
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for ( int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point ) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId = _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd() ) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;

        }

        ++_out_MeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedConeWithSavedConeITKFileTest, vtk2itkConeFixture){

    BOOST_CHECK_EQUAL(_SavedPoints.size(), _out_Mesh->GetNumberOfPoints());

    //Check equivalency of points
    for ( size_t pnt_id = 0; pnt_id < _out_Mesh->GetNumberOfPoints(); ++pnt_id) {
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[0], _SavedPoints[pnt_id].x);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[1], _SavedPoints[pnt_id].y);
        volcart::testing::SmallOrClose(_out_Mesh->GetPoint(pnt_id)[2], _SavedPoints[pnt_id].z);
    }

    // Normals //
    ITKPointIterator point = _out_Mesh->GetPoints()->Begin();
    for ( int p = 0; point != _out_Mesh->GetPoints()->End(); ++p, ++point ) {
        ITKPixel _out_MeshNormal;
        _out_Mesh->GetPointData(point.Index(), &_out_MeshNormal);

        volcart::testing::SmallOrClose(_out_MeshNormal[0], _SavedPoints[p].nx);
        volcart::testing::SmallOrClose(_out_MeshNormal[1], _SavedPoints[p].ny);
        volcart::testing::SmallOrClose(_out_MeshNormal[2], _SavedPoints[p].nz);
    }

    ITKCellIterator _out_MeshCell = _out_Mesh->GetCells()->Begin();
    int c = 0;

    while (_out_MeshCell != _out_Mesh->GetCells()->End()) {

        ITKPointInCellIterator _out_MeshPointId = _out_MeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_MeshPointId != _out_MeshCell.Value()->PointIdsEnd() ) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_MeshPointId, _SavedCells[c].v3);

            _out_MeshPointId++;
            counter++;

        }

        ++_out_MeshCell;
        ++c;
    }

}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/*                                                                                                                    */
/*                                                EDGE CASE TESTS                                                     */
/*                                                                                                                    */
/**********************************************************************************************************************/
/**********************************************************************************************************************/

// Test whether things fail if the meshes don't have normals
BOOST_FIXTURE_TEST_CASE( MeshWithNoNormals, NoNormalsFixture ) {
    // ITK to VTK
    vtkSmartPointer<vtkPolyData> vtk_Mesh =  vtkSmartPointer<vtkPolyData>::New();
    volcart::meshing::itk2vtk( _itk_Mesh, vtk_Mesh );

    // VTK to ITK
    ITKMesh::Pointer itk_Mesh2 = ITKMesh::New();
    volcart::meshing::vtk2itk( vtk_Mesh, itk_Mesh2 );

    // ITK to ITK Quad
    volcart::QuadMesh::Pointer itk_Quad = volcart::QuadMesh::New();
    volcart::meshing::itk2itkQE( itk_Mesh2, itk_Quad );

    // ITK Quad to ITK
    itk_Mesh2 = ITKMesh::New();
    volcart::meshing::itkQE2itk( itk_Quad, itk_Mesh2 );
}
