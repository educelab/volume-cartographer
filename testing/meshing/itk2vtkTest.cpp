//
// Created by Ryan Taber on 10/14/15.
//

#ifndef VC_BOOST_STATIC_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE itk2vtk

#include <boost/test/unit_test.hpp>

#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "shapes.h"
#include "itk2vtk.h"
#include "parsingHelpers.h"
#include <itkMeshFileReader.h>


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
        _in_PlaneITKMesh = _Plane.itkMesh();
                
        //init vtkpolydata pointers
        _out_PlaneVTKMesh = vtkPolyData::New();
        
        //call itk2vtk 
        volcart::meshing::itk2vtk(_in_PlaneITKMesh, _out_PlaneVTKMesh);

        //Read in the vtk.ply file
        volcart::testing::ParsingHelpers::parsePlyFile("ITKPlaneMeshConvertedToVTK.ply", 
                                                                            _SavedVTKPlanePoints, _SavedVTKPlaneCells);
        
        std::cerr << "setting up itk2vtk Plane test objects" << std::endl;
    }

    ~itk2vtkPlaneFixture(){ std::cerr << "cleaning up itk2vtk Plane test objects" << std::endl; }

    //init Plane
    volcart::shapes::Plane _Plane;

    //declare itk input mesh object
    VC_MeshType::Pointer _in_PlaneITKMesh;
    
    //declare vtk output polydata object
    vtkPolyData* _out_PlaneVTKMesh;
    
    //Declare vectors that will hold read data created by itk2vtkExample.cpp
    std::vector<VC_Vertex> _SavedVTKPlanePoints;
    std::vector<VC_Cell> _SavedVTKPlaneCells;
};

struct itk2vtkCubeFixture {

    itk2vtkCubeFixture() {
        
        _in_CubeITKMesh = _Cube.itkMesh();
        _out_CubeVTKMesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_CubeITKMesh, _out_CubeVTKMesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKCubeMeshConvertedToVTK.ply",
                                                       _SavedVTKCubePoints, _SavedVTKCubeCells);

        std::cerr << "setting up itk2vtk Cube test objects" << std::endl;
    }

    ~itk2vtkCubeFixture(){ std::cerr << "cleaning up itk2vtk Cube test objects" << std::endl; }
    
    volcart::shapes::Cube _Cube;
    VC_MeshType::Pointer _in_CubeITKMesh;
    vtkPolyData* _out_CubeVTKMesh;
    std::vector<VC_Vertex> _SavedVTKCubePoints;
    std::vector<VC_Cell> _SavedVTKCubeCells;

};


struct itk2vtkArchFixture {

    itk2vtkArchFixture() {

        _in_ArchITKMesh = _Arch.itkMesh();
        _out_ArchVTKMesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_ArchITKMesh, _out_ArchVTKMesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKArchMeshConvertedToVTK.ply",
                                                       _SavedVTKArchPoints, _SavedVTKArchCells);

        std::cerr << "setting up itk2vtk Arch test objects" << std::endl;
    }

    ~itk2vtkArchFixture(){ std::cerr << "cleaning up itk2vtk Arch test objects" << std::endl; }

    volcart::shapes::Arch _Arch;
    VC_MeshType::Pointer _in_ArchITKMesh;
    vtkPolyData* _out_ArchVTKMesh;
    std::vector<VC_Vertex> _SavedVTKArchPoints;
    std::vector<VC_Cell> _SavedVTKArchCells;

};

struct itk2vtkSphereFixture {

    itk2vtkSphereFixture() {

        _in_SphereITKMesh = _Sphere.itkMesh();
        _out_SphereVTKMesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_SphereITKMesh, _out_SphereVTKMesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKSphereMeshConvertedToVTK.ply",
                                                       _SavedVTKSpherePoints, _SavedVTKSphereCells);

        std::cerr << "setting up itk2vtk Sphere test objects" << std::endl;
    }

    ~itk2vtkSphereFixture(){ std::cerr << "cleaning up itk2vtk Sphere test objects" << std::endl; }

    volcart::shapes::Sphere _Sphere;
    VC_MeshType::Pointer _in_SphereITKMesh;
    vtkPolyData* _out_SphereVTKMesh;
    std::vector<VC_Vertex> _SavedVTKSpherePoints;
    std::vector<VC_Cell> _SavedVTKSphereCells;

};

struct itk2vtkConeFixture {

    itk2vtkConeFixture() {

        _in_ConeITKMesh = _Cone.itkMesh();
        _out_ConeVTKMesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(_in_ConeITKMesh, _out_ConeVTKMesh);
        volcart::testing::ParsingHelpers::parsePlyFile("ITKConeMeshConvertedToVTK.ply",
                                                       _SavedVTKConePoints, _SavedVTKConeCells);

        std::cerr << "setting up itk2vtk Cone test objects" << std::endl;
    }

    ~itk2vtkConeFixture(){ std::cerr << "cleaning up itk2vtk Cone test objects" << std::endl; }

    volcart::shapes::Cone _Cone;
    VC_MeshType::Pointer _in_ConeITKMesh;
    vtkPolyData* _out_ConeVTKMesh;
    std::vector<VC_Vertex> _SavedVTKConePoints;
    std::vector<VC_Cell> _SavedVTKConeCells;

};

/*
 * The following five fixture build test inputs for the vtk2itk tests
 * 
 * Comments are included in the Plane Fixture only...the remainder of the fixure copy the same ideas
 */

struct vtk2itkPlaneFixture {

    vtk2itkPlaneFixture() {

        _in_PlaneVTKMesh = _Plane.vtkMesh();
        _out_PlaneITKMesh = VC_MeshType::New();
        
        // assign smartpointer data into vtkPolyData* object
        vtkReadPlaneData = _in_PlaneVTKMesh.GetPointer();

        // convert from vtk mesh to itk mesh
        volcart::meshing::vtk2itk(vtkReadPlaneData, _out_PlaneITKMesh);

        // read data in from saved file created by itk2vtkExample.cpp
        volcart::testing::ParsingHelpers::parseObjFile("VTKPlaneMeshConvertedToITK.obj", 
                                                                           _SavedITKPlanePoints, _SavedITKPlaneCells);
        
        std::cerr << "setting up vtk2itk Plane test objects" << std::endl;
    }

    ~vtk2itkPlaneFixture(){ std::cerr << "cleaning up vtk2itk Plane test objects" << std::endl; }

    //init shape
    volcart::shapes::Plane _Plane;

    //declare input vtkPolyData pointer to hold input mesh data
    vtkSmartPointer<vtkPolyData> _in_PlaneVTKMesh;

    //pointer that will hold input mesh data for conversion to itk mesh
    vtkPolyData* vtkReadPlaneData;
    
    //declare mesh to hold output of vtk2itk call
    VC_MeshType::Pointer _out_PlaneITKMesh;

    //declare vectors to hold points and cells from savedITK data file created by itk2vtkExample.cpp
    std::vector<VC_Vertex> _SavedITKPlanePoints;
    std::vector<VC_Cell> _SavedITKPlaneCells;
    
};

struct vtk2itkCubeFixture {

    vtk2itkCubeFixture() {

        _in_CubeVTKMesh = _Cube.vtkMesh();
        _out_CubeITKMesh = VC_MeshType::New();
        vtkReadCubeData = _in_CubeVTKMesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadCubeData, _out_CubeITKMesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKCubeMeshConvertedToITK.obj",
                                                       _SavedITKCubePoints, _SavedITKCubeCells);

        std::cerr << "setting up vtk2itk Cube test objects" << std::endl;
    }

    ~vtk2itkCubeFixture(){ std::cerr << "cleaning up vtk2itk Cube test objects" << std::endl; }
    
    volcart::shapes::Cube _Cube;
    vtkSmartPointer<vtkPolyData> _in_CubeVTKMesh;
    vtkPolyData* vtkReadCubeData;
    VC_MeshType::Pointer _out_CubeITKMesh;
    std::vector<VC_Vertex> _SavedITKCubePoints;
    std::vector<VC_Cell> _SavedITKCubeCells;

};


struct vtk2itkArchFixture {

    vtk2itkArchFixture() {

        _in_ArchVTKMesh = _Arch.vtkMesh();
        _out_ArchITKMesh = VC_MeshType::New();
        vtkReadArchData = _in_ArchVTKMesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadArchData, _out_ArchITKMesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKArchMeshConvertedToITK.obj",
                                                       _SavedITKArchPoints, _SavedITKArchCells);

        std::cerr << "setting up vtk2itk Arch test objects" << std::endl;
    }

    ~vtk2itkArchFixture(){ std::cerr << "cleaning up vtk2itk Arch test objects" << std::endl; }

    volcart::shapes::Arch _Arch;
    vtkSmartPointer<vtkPolyData> _in_ArchVTKMesh;
    vtkPolyData* vtkReadArchData;
    VC_MeshType::Pointer _out_ArchITKMesh;
    std::vector<VC_Vertex> _SavedITKArchPoints;
    std::vector<VC_Cell> _SavedITKArchCells;

};

struct vtk2itkSphereFixture {

    vtk2itkSphereFixture() {

        _in_SphereVTKMesh = _Sphere.vtkMesh();
        _out_SphereITKMesh = VC_MeshType::New();
        vtkReadSphereData = _in_SphereVTKMesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadSphereData, _out_SphereITKMesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKSphereMeshConvertedToITK.obj",
                                                       _SavedITKSpherePoints, _SavedITKSphereCells);

        std::cerr << "setting up vtk2itk Sphere test objects" << std::endl;
    }

    ~vtk2itkSphereFixture(){ std::cerr << "cleaning up vtk2itk Sphere test objects" << std::endl; }

    volcart::shapes::Sphere _Sphere;
    vtkSmartPointer<vtkPolyData> _in_SphereVTKMesh;
    vtkPolyData* vtkReadSphereData;
    VC_MeshType::Pointer _out_SphereITKMesh;
    std::vector<VC_Vertex> _SavedITKSpherePoints;
    std::vector<VC_Cell> _SavedITKSphereCells;

};

struct vtk2itkConeFixture {

    vtk2itkConeFixture() {

        _in_ConeVTKMesh = _Cone.vtkMesh();
        _out_ConeITKMesh = VC_MeshType::New();
        vtkReadConeData = _in_ConeVTKMesh.GetPointer();
        volcart::meshing::vtk2itk(vtkReadConeData, _out_ConeITKMesh);
        volcart::testing::ParsingHelpers::parseObjFile("VTKConeMeshConvertedToITK.obj",
                                                       _SavedITKConePoints, _SavedITKConeCells);

        std::cerr << "setting up vtk2itk Cone test objects" << std::endl;
    }

    ~vtk2itkConeFixture(){ std::cerr << "cleaning up vtk2itk Cone test objects" << std::endl; }

    volcart::shapes::Cone _Cone;
    vtkSmartPointer<vtkPolyData> _in_ConeVTKMesh;
    vtkPolyData* vtkReadConeData;
    VC_MeshType::Pointer _out_ConeITKMesh;
    std::vector<VC_Vertex> _SavedITKConePoints;
    std::vector<VC_Cell> _SavedITKConeCells;

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
    BOOST_CHECK_EQUAL( _out_PlaneVTKMesh->GetNumberOfPoints(), _SavedVTKPlanePoints.size() );

    std::cerr << "Comparing points and normals..." << std::endl;
    //Now iterate over point sets and compare coordinate and normal values
    for ( size_t pnt_id = 0; pnt_id < _out_PlaneVTKMesh ->GetNumberOfPoints(); ++pnt_id) {

        BOOST_CHECK_EQUAL( _out_PlaneVTKMesh->GetPoint(pnt_id)[0], _SavedVTKPlanePoints[pnt_id].x);
        BOOST_CHECK_EQUAL( _out_PlaneVTKMesh->GetPoint(pnt_id)[1], _SavedVTKPlanePoints[pnt_id].y);
        BOOST_CHECK_EQUAL( _out_PlaneVTKMesh->GetPoint(pnt_id)[2], _SavedVTKPlanePoints[pnt_id].z);
        
    }

    //Normals
    vtkDataArray *out_PlanePointNormals = _out_PlaneVTKMesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_PlaneVTKMesh->GetNumberOfPoints(); ++pnt_id ) {

        VC_PixelType out_PlaneSingleIdNormalTuple = out_PlanePointNormals->GetTuple(pnt_id);

        BOOST_CHECK_EQUAL( out_PlaneSingleIdNormalTuple[0], _SavedVTKPlanePoints[pnt_id].nx);
        BOOST_CHECK_EQUAL( out_PlaneSingleIdNormalTuple[1], _SavedVTKPlanePoints[pnt_id].ny);
        BOOST_CHECK_EQUAL( out_PlaneSingleIdNormalTuple[2], _SavedVTKPlanePoints[pnt_id].nz);
    }

    //Cells

    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(_out_PlaneVTKMesh->GetNumberOfCells(), _SavedVTKPlaneCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_PlaneVTKMesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKPlaneCell = _out_PlaneVTKMesh->GetCell(c_id);

        //Check that vertices making up the corresponding cells are equal
        //We have three checks since there are three points that make up a cell
        BOOST_CHECK_EQUAL(out_VTKPlaneCell->GetPointIds()->GetId(0), _SavedVTKPlaneCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKPlaneCell->GetPointIds()->GetId(1), _SavedVTKPlaneCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKPlaneCell->GetPointIds()->GetId(2), _SavedVTKPlaneCells[c_id].v3);

    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedCubeWithSavedCubeVTKFileTest, itk2vtkCubeFixture){

    BOOST_CHECK_EQUAL( _out_CubeVTKMesh->GetNumberOfPoints(), _SavedVTKCubePoints.size() );

    for ( size_t pnt_id = 0; pnt_id < _out_CubeVTKMesh ->GetNumberOfPoints(); ++pnt_id) {

        BOOST_CHECK_EQUAL( _out_CubeVTKMesh->GetPoint(pnt_id)[0], _SavedVTKCubePoints[pnt_id].x);
        BOOST_CHECK_EQUAL( _out_CubeVTKMesh->GetPoint(pnt_id)[1], _SavedVTKCubePoints[pnt_id].y);
        BOOST_CHECK_EQUAL( _out_CubeVTKMesh->GetPoint(pnt_id)[2], _SavedVTKCubePoints[pnt_id].z);
    }
    
    vtkDataArray *out_CubePointNormals = _out_CubeVTKMesh->GetPointData()->GetNormals();
    for ( int pnt_id = 0; pnt_id < _out_CubeVTKMesh->GetNumberOfPoints(); ++pnt_id ) {

        VC_PixelType out_CubeSingleIdNormalTuple = out_CubePointNormals->GetTuple(pnt_id);

        BOOST_CHECK_EQUAL( out_CubeSingleIdNormalTuple[0], _SavedVTKCubePoints[pnt_id].nx);
        BOOST_CHECK_EQUAL( out_CubeSingleIdNormalTuple[1], _SavedVTKCubePoints[pnt_id].ny);
        BOOST_CHECK_EQUAL( out_CubeSingleIdNormalTuple[2], _SavedVTKCubePoints[pnt_id].nz);
    }
    
    BOOST_CHECK_EQUAL(_out_CubeVTKMesh->GetNumberOfCells(), _SavedVTKCubeCells.size());

    for ( int c_id = 0; c_id < _out_CubeVTKMesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKCubeCell = _out_CubeVTKMesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKCubeCell->GetPointIds()->GetId(0), _SavedVTKCubeCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKCubeCell->GetPointIds()->GetId(1), _SavedVTKCubeCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKCubeCell->GetPointIds()->GetId(2), _SavedVTKCubeCells[c_id].v3);

    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedArchWithSavedArchVTKFileTest, itk2vtkArchFixture){

    BOOST_CHECK_EQUAL( _out_ArchVTKMesh->GetNumberOfPoints(), _SavedVTKArchPoints.size() );

    for ( size_t pnt_id = 0; pnt_id < _out_ArchVTKMesh ->GetNumberOfPoints(); ++pnt_id) {

        BOOST_CHECK_CLOSE_FRACTION( _out_ArchVTKMesh->GetPoint(pnt_id)[0], _SavedVTKArchPoints[pnt_id].x, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( _out_ArchVTKMesh->GetPoint(pnt_id)[1], _SavedVTKArchPoints[pnt_id].y, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( _out_ArchVTKMesh->GetPoint(pnt_id)[2], _SavedVTKArchPoints[pnt_id].z, 0.00001);
    }

    vtkDataArray *out_ArchPointNormals = _out_ArchVTKMesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_ArchVTKMesh->GetNumberOfPoints(); ++pnt_id ) {

        VC_PixelType out_ArchSingleIdNormalTuple = out_ArchPointNormals->GetTuple(pnt_id);

        BOOST_CHECK_CLOSE_FRACTION( out_ArchSingleIdNormalTuple[0], _SavedVTKArchPoints[pnt_id].nx, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( out_ArchSingleIdNormalTuple[1], _SavedVTKArchPoints[pnt_id].ny, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( out_ArchSingleIdNormalTuple[2], _SavedVTKArchPoints[pnt_id].nz, 0.00001);
    }

    BOOST_CHECK_EQUAL(_out_ArchVTKMesh->GetNumberOfCells(), _SavedVTKArchCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_ArchVTKMesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKArchCell = _out_ArchVTKMesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKArchCell->GetPointIds()->GetId(0), _SavedVTKArchCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKArchCell->GetPointIds()->GetId(1), _SavedVTKArchCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKArchCell->GetPointIds()->GetId(2), _SavedVTKArchCells[c_id].v3);

    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedSphereWithSavedSphereVTKFileTest, itk2vtkSphereFixture){

    BOOST_CHECK_EQUAL( _out_SphereVTKMesh->GetNumberOfPoints(), _SavedVTKSpherePoints.size() );

    for ( size_t pnt_id = 0; pnt_id < _out_SphereVTKMesh ->GetNumberOfPoints(); ++pnt_id) {

        BOOST_CHECK_CLOSE_FRACTION( _out_SphereVTKMesh->GetPoint(pnt_id)[0], _SavedVTKSpherePoints[pnt_id].x, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( _out_SphereVTKMesh->GetPoint(pnt_id)[1], _SavedVTKSpherePoints[pnt_id].y, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( _out_SphereVTKMesh->GetPoint(pnt_id)[2], _SavedVTKSpherePoints[pnt_id].z, 0.00001);
    }

    vtkDataArray *out_SpherePointNormals = _out_SphereVTKMesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_SphereVTKMesh->GetNumberOfPoints(); ++pnt_id ) {

        VC_PixelType out_SphereSingleIdNormalTuple = out_SpherePointNormals->GetTuple(pnt_id);

        BOOST_CHECK_CLOSE_FRACTION( out_SphereSingleIdNormalTuple[0], _SavedVTKSpherePoints[pnt_id].nx, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( out_SphereSingleIdNormalTuple[1], _SavedVTKSpherePoints[pnt_id].ny, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( out_SphereSingleIdNormalTuple[2], _SavedVTKSpherePoints[pnt_id].nz, 0.00001);
    }

    BOOST_CHECK_EQUAL(_out_SphereVTKMesh->GetNumberOfCells(), _SavedVTKSphereCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_SphereVTKMesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKSphereCell = _out_SphereVTKMesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKSphereCell->GetPointIds()->GetId(0), _SavedVTKSphereCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKSphereCell->GetPointIds()->GetId(1), _SavedVTKSphereCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKSphereCell->GetPointIds()->GetId(2), _SavedVTKSphereCells[c_id].v3);

    }
}

BOOST_FIXTURE_TEST_CASE(CompareFixtureITKToVTKConvertedConeWithSavedConeVTKFileTest, itk2vtkConeFixture){

    BOOST_CHECK_EQUAL( _out_ConeVTKMesh->GetNumberOfPoints(), _SavedVTKConePoints.size() );

    std::cerr << "Comparing points and normals..." << std::endl;
    for ( size_t pnt_id = 0; pnt_id < _out_ConeVTKMesh ->GetNumberOfPoints(); ++pnt_id) {

        BOOST_CHECK_CLOSE_FRACTION( _out_ConeVTKMesh->GetPoint(pnt_id)[0], _SavedVTKConePoints[pnt_id].x, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( _out_ConeVTKMesh->GetPoint(pnt_id)[1], _SavedVTKConePoints[pnt_id].y, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION( _out_ConeVTKMesh->GetPoint(pnt_id)[2], _SavedVTKConePoints[pnt_id].z, 0.00001);
    }

    vtkDataArray *out_ConePointNormals = _out_ConeVTKMesh->GetPointData()->GetNormals();
    for ( vtkIdType pnt_id = 0; pnt_id < _out_ConeVTKMesh->GetNumberOfPoints(); ++pnt_id ) {

        VC_PixelType out_ConeSingleIdNormalTuple = out_ConePointNormals->GetTuple(pnt_id);

        //added tolerance value
        BOOST_CHECK_CLOSE( out_ConeSingleIdNormalTuple[0], _SavedVTKConePoints[pnt_id].nx, 0.1);
        BOOST_CHECK_CLOSE( out_ConeSingleIdNormalTuple[1], _SavedVTKConePoints[pnt_id].ny, 0.1);
        BOOST_CHECK_CLOSE( out_ConeSingleIdNormalTuple[2], _SavedVTKConePoints[pnt_id].nz, 0.1);
    }

    BOOST_CHECK_EQUAL(_out_ConeVTKMesh->GetNumberOfCells(), _SavedVTKConeCells.size());

    for ( vtkIdType c_id = 0; c_id < _out_ConeVTKMesh->GetNumberOfCells(); c_id++){

        vtkCell *out_VTKConeCell = _out_ConeVTKMesh->GetCell(c_id);

        BOOST_CHECK_EQUAL(out_VTKConeCell->GetPointIds()->GetId(0), _SavedVTKConeCells[c_id].v1);
        BOOST_CHECK_EQUAL(out_VTKConeCell->GetPointIds()->GetId(1), _SavedVTKConeCells[c_id].v2);
        BOOST_CHECK_EQUAL(out_VTKConeCell->GetPointIds()->GetId(2), _SavedVTKConeCells[c_id].v3);

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
    BOOST_CHECK_EQUAL(_SavedITKPlanePoints.size(), _out_PlaneITKMesh->GetNumberOfPoints());

    //Check equivalency of points
    for ( size_t pnt_id = 0; pnt_id < _out_PlaneITKMesh->GetNumberOfPoints(); ++pnt_id) {

        //check the points in both vtk meshes
        BOOST_CHECK_EQUAL(_SavedITKPlanePoints[pnt_id].x, _out_PlaneITKMesh->GetPoint(pnt_id)[0]);
        BOOST_CHECK_EQUAL(_SavedITKPlanePoints[pnt_id].y, _out_PlaneITKMesh->GetPoint(pnt_id)[1]);
        BOOST_CHECK_EQUAL(_SavedITKPlanePoints[pnt_id].z, _out_PlaneITKMesh->GetPoint(pnt_id)[2]);
    }

    // Normals //
    int p =0;
    VC_PointsInMeshIterator point = _out_PlaneITKMesh->GetPoints()->Begin();
    for ( ; point != _out_PlaneITKMesh->GetPoints()->End(); ++point ) {

        VC_PixelType _out_PlaneITKMeshNormal;
        _out_PlaneITKMesh->GetPointData(point.Index(), &_out_PlaneITKMeshNormal);

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(_out_PlaneITKMeshNormal[0], _SavedITKPlanePoints[p].nx);
        BOOST_CHECK_EQUAL(_out_PlaneITKMeshNormal[1], _SavedITKPlanePoints[p].ny);
        BOOST_CHECK_EQUAL(_out_PlaneITKMeshNormal[2], _SavedITKPlanePoints[p].nz);

        ++p;
    }
    
    //Cells (faces)

    // Initialize Cell Iterators
    VC_CellIterator _out_PlaneITKMeshCell = _out_PlaneITKMesh->GetCells()->Begin();

    int c = 0;

    while (_out_PlaneITKMeshCell != _out_PlaneITKMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator _out_PlaneITKMeshPointId = _out_PlaneITKMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( _out_PlaneITKMeshPointId != _out_PlaneITKMeshCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_PlaneITKMeshPointId, _SavedITKPlaneCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_PlaneITKMeshPointId, _SavedITKPlaneCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_PlaneITKMeshPointId, _SavedITKPlaneCells[c].v3);

            //increment points
            _out_PlaneITKMeshPointId++;
            counter++;

        }

        //increment cells
        ++_out_PlaneITKMeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedCubeWithSavedCubeITKFileTest, vtk2itkCubeFixture){
    
    BOOST_CHECK_EQUAL(_SavedITKCubePoints.size(), _out_CubeITKMesh->GetNumberOfPoints());
    
    for ( size_t pnt_id = 0; pnt_id < _out_CubeITKMesh->GetNumberOfPoints(); ++pnt_id) {
        
        BOOST_CHECK_EQUAL(_SavedITKCubePoints[pnt_id].x, _out_CubeITKMesh->GetPoint(pnt_id)[0]);
        BOOST_CHECK_EQUAL(_SavedITKCubePoints[pnt_id].y, _out_CubeITKMesh->GetPoint(pnt_id)[1]);
        BOOST_CHECK_EQUAL(_SavedITKCubePoints[pnt_id].z, _out_CubeITKMesh->GetPoint(pnt_id)[2]);
    }
    
    int p =0;
    VC_PointsInMeshIterator point = _out_CubeITKMesh->GetPoints()->Begin();
    for ( ; point != _out_CubeITKMesh->GetPoints()->End(); ++point ) {

        VC_PixelType _out_CubeITKMeshNormal;
        _out_CubeITKMesh->GetPointData(point.Index(), &_out_CubeITKMeshNormal);
        
        BOOST_CHECK_EQUAL(_out_CubeITKMeshNormal[0], _SavedITKCubePoints[p].nx);
        BOOST_CHECK_EQUAL(_out_CubeITKMeshNormal[1], _SavedITKCubePoints[p].ny);
        BOOST_CHECK_EQUAL(_out_CubeITKMeshNormal[2], _SavedITKCubePoints[p].nz);

        ++p;
    }

    VC_CellIterator _out_CubeITKMeshCell = _out_CubeITKMesh->GetCells()->Begin();
    int c = 0;

    while (_out_CubeITKMeshCell != _out_CubeITKMesh->GetCells()->End()) {
        
        VC_PointsInCellIterator _out_CubeITKMeshPointId = _out_CubeITKMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_CubeITKMeshPointId != _out_CubeITKMeshCell.Value()->PointIdsEnd() ) {
            
            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_CubeITKMeshPointId, _SavedITKCubeCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_CubeITKMeshPointId, _SavedITKCubeCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_CubeITKMeshPointId, _SavedITKCubeCells[c].v3);
            
            _out_CubeITKMeshPointId++;
            counter++;
        }
        
        ++_out_CubeITKMeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedArchWithSavedArchITKFileTest, vtk2itkArchFixture){
    
    BOOST_CHECK_EQUAL(_SavedITKArchPoints.size(), _out_ArchITKMesh->GetNumberOfPoints());
    
    for ( size_t pnt_id = 0; pnt_id < _out_ArchITKMesh->GetNumberOfPoints(); ++pnt_id) {
        
        BOOST_CHECK_CLOSE_FRACTION(_SavedITKArchPoints[pnt_id].x, _out_ArchITKMesh->GetPoint(pnt_id)[0], 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_SavedITKArchPoints[pnt_id].y, _out_ArchITKMesh->GetPoint(pnt_id)[1], 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_SavedITKArchPoints[pnt_id].z, _out_ArchITKMesh->GetPoint(pnt_id)[2], 0.00001);
    }
    
    int p =0;
    VC_PointsInMeshIterator point = _out_ArchITKMesh->GetPoints()->Begin();
    for ( ; point != _out_ArchITKMesh->GetPoints()->End(); ++point ) {

        VC_PixelType _out_ArchITKMeshNormal;
        _out_ArchITKMesh->GetPointData(point.Index(), &_out_ArchITKMeshNormal);
        
        BOOST_CHECK_EQUAL(_out_ArchITKMeshNormal[0], _SavedITKArchPoints[p].nx);
        BOOST_CHECK_EQUAL(_out_ArchITKMeshNormal[1], _SavedITKArchPoints[p].ny);
        BOOST_CHECK_EQUAL(_out_ArchITKMeshNormal[2], _SavedITKArchPoints[p].nz);

        ++p;
    }

    VC_CellIterator _out_ArchITKMeshCell = _out_ArchITKMesh->GetCells()->Begin();
    int c = 0;

    while (_out_ArchITKMeshCell != _out_ArchITKMesh->GetCells()->End()) {
        
        VC_PointsInCellIterator _out_ArchITKMeshPointId = _out_ArchITKMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_ArchITKMeshPointId != _out_ArchITKMeshCell.Value()->PointIdsEnd() ) {
            
            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_ArchITKMeshPointId, _SavedITKArchCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_ArchITKMeshPointId, _SavedITKArchCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_ArchITKMeshPointId, _SavedITKArchCells[c].v3);
            
            _out_ArchITKMeshPointId++;
            counter++;

        }
        
        ++_out_ArchITKMeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedSphereWithSavedSphereITKFileTest, vtk2itkSphereFixture){

    BOOST_CHECK_EQUAL(_SavedITKSpherePoints.size(), _out_SphereITKMesh->GetNumberOfPoints());

    for ( size_t pnt_id = 0; pnt_id < _out_SphereITKMesh->GetNumberOfPoints(); ++pnt_id) {

        BOOST_CHECK_CLOSE_FRACTION(_SavedITKSpherePoints[pnt_id].x, _out_SphereITKMesh->GetPoint(pnt_id)[0], 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_SavedITKSpherePoints[pnt_id].y, _out_SphereITKMesh->GetPoint(pnt_id)[1], 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_SavedITKSpherePoints[pnt_id].z, _out_SphereITKMesh->GetPoint(pnt_id)[2], 0.00001);
    }

    int p =0;
    VC_PointsInMeshIterator point = _out_SphereITKMesh->GetPoints()->Begin();
    for ( ; point != _out_SphereITKMesh->GetPoints()->End(); ++point ) {

        VC_PixelType _out_SphereITKMeshNormal;
        _out_SphereITKMesh->GetPointData(point.Index(), &_out_SphereITKMeshNormal);

        BOOST_CHECK_EQUAL(_out_SphereITKMeshNormal[0], _SavedITKSpherePoints[p].nx);
        BOOST_CHECK_EQUAL(_out_SphereITKMeshNormal[1], _SavedITKSpherePoints[p].ny);
        BOOST_CHECK_EQUAL(_out_SphereITKMeshNormal[2], _SavedITKSpherePoints[p].nz);

        ++p;
    }

    VC_CellIterator _out_SphereITKMeshCell = _out_SphereITKMesh->GetCells()->Begin();
    int c = 0;

    while (_out_SphereITKMeshCell != _out_SphereITKMesh->GetCells()->End()) {

        VC_PointsInCellIterator _out_SphereITKMeshPointId = _out_SphereITKMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_SphereITKMeshPointId != _out_SphereITKMeshCell.Value()->PointIdsEnd() ) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_SphereITKMeshPointId, _SavedITKSphereCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_SphereITKMeshPointId, _SavedITKSphereCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_SphereITKMeshPointId, _SavedITKSphereCells[c].v3);

            _out_SphereITKMeshPointId++;
            counter++;

        }

        ++_out_SphereITKMeshCell;
        ++c;
    }

}

BOOST_FIXTURE_TEST_CASE(CompareFixtureVTKToITKConvertedConeWithSavedConeITKFileTest, vtk2itkConeFixture){

    BOOST_CHECK_EQUAL(_SavedITKConePoints.size(), _out_ConeITKMesh->GetNumberOfPoints());

    for ( size_t pnt_id = 0; pnt_id < _out_ConeITKMesh->GetNumberOfPoints(); ++pnt_id) {

        BOOST_CHECK_CLOSE_FRACTION(_SavedITKConePoints[pnt_id].x, _out_ConeITKMesh->GetPoint(pnt_id)[0], 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_SavedITKConePoints[pnt_id].y, _out_ConeITKMesh->GetPoint(pnt_id)[1], 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_SavedITKConePoints[pnt_id].z, _out_ConeITKMesh->GetPoint(pnt_id)[2], 0.00001);
    }

    int p =0;
    VC_PointsInMeshIterator point = _out_ConeITKMesh->GetPoints()->Begin();
    for ( ; point != _out_ConeITKMesh->GetPoints()->End(); ++point ) {

        VC_PixelType _out_ConeITKMeshNormal;
        _out_ConeITKMesh->GetPointData(point.Index(), &_out_ConeITKMeshNormal);

        BOOST_CHECK_CLOSE_FRACTION(_out_ConeITKMeshNormal[0], _SavedITKConePoints[p].nx, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_out_ConeITKMeshNormal[1], _SavedITKConePoints[p].ny, 0.00001);
        BOOST_CHECK_CLOSE_FRACTION(_out_ConeITKMeshNormal[2], _SavedITKConePoints[p].nz, 0.00001);

        ++p;
    }

    VC_CellIterator _out_ConeITKMeshCell = _out_ConeITKMesh->GetCells()->Begin();
    int c = 0;

    while (_out_ConeITKMeshCell != _out_ConeITKMesh->GetCells()->End()) {

        VC_PointsInCellIterator _out_ConeITKMeshPointId = _out_ConeITKMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        while ( _out_ConeITKMeshPointId != _out_ConeITKMeshCell.Value()->PointIdsEnd() ) {

            if (counter == 0)
                BOOST_CHECK_EQUAL(*_out_ConeITKMeshPointId, _SavedITKConeCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*_out_ConeITKMeshPointId, _SavedITKConeCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*_out_ConeITKMeshPointId, _SavedITKConeCells[c].v3);

            _out_ConeITKMeshPointId++;
            counter++;

        }

        ++_out_ConeITKMeshCell;
        ++c;
    }

}