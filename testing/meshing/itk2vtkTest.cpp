//
// Created by Ryan Taber on 10/14/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE itk2vtk

#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <vc_defines.h>
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
 *           common/shapes/Plane.h, can be written into                             *
 *                                                                                  *
 *                                                                                  *
 *  This file is broken up into two test fixtures (ivFix & viFix) which initialize  *
 *  the objects used in each of the two test cases.                                 *
 *                                                                                  *
 *  i2v (test case):                                                                *
 *                                                                                  *
 *      Takes an itk mesh created from fixture and data to vtkpolydata pointer.     *
 *      Vtkpolydata then converted back to itk mesh. Successful test if converted   *
 *      itk matches original itk mesh data points                                   *
 *                                                                                  *
 *  v2i (test case):                                                                *
 *                                                                                  *
 *      Same idea as i2v test case except that the test starts with vtkpolydata.    *
 *      The vtk data is converted into itk mesh and back to vtk. Success if the     *
 *      original vtk and converted vtk data points match.                           *
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
 * This builds objects for the i2v case below that reference
 * the fixture as their second argument
 *
 */

struct ivFix {

    ivFix() {

        _mesh = mesh.itkMesh();
        _vtk = vtkPolyData::New();

        std::cerr << "setting up itk2vtk objects" << std::endl;
    }

    ~ivFix(){ std::cerr << "cleaning up itk2vtk objects" << std::endl; }

    VC_MeshType::Pointer _mesh;
    volcart::shapes::Plane mesh;
    vtkPolyData* _vtk;
};

/*
 * This builds objects for the v2i test case below that reference
 * the fixture as their second argument
 *
 */


struct viFix {

    viFix() {

        vtk = mesh.vtkMesh();
        itkMesh = VC_MeshType::New();

        std::cerr << "setting up vtk2itk objects" << std::endl;
    }

    ~viFix(){ std::cerr << "cleaning up vtk2itk objects" << std::endl; }

    VC_MeshType::Pointer itkMesh;
    volcart::shapes::Plane mesh;
    vtkSmartPointer<vtkPolyData> vtk;
};


//
// check successful conversion from itk mesh to vtk mesh
//
// Note: _mesh is an itk mesh created in ivFix
//       _vtk is a vtk mesh created in ivFix
//

BOOST_FIXTURE_TEST_CASE(i2v, ivFix){

    // initialize a new VC_MeshType::Pointer
    // this will hold the values for the new itk mesh that
    // is to be compared with _mesh.
    VC_MeshType::Pointer _itk = VC_MeshType::New();

    // convert from itk mesh to vtk and back again
    volcart::meshing::itk2vtk(_mesh, _vtk);
    volcart::meshing::vtk2itk(_vtk, _itk);


    /* Now the original _mesh and the converted _itk
     * mesh should contain the same data. The following three for loops
     * check to see if this is true...and if the itk2vtk() constructor functions
     * properly. Uncomment the std<<cerr lines to see the example data being tested.
     */

    // Points //

    //Confirm equal number of points in both meshes
    BOOST_CHECK_EQUAL(_mesh->GetNumberOfPoints(), _itk->GetNumberOfPoints());


    //std::cerr << "Points:" << std::endl;
    for ( size_t p_id = 0; p_id < _mesh->GetNumberOfPoints(); ++p_id) {

        //check the points in both vtk meshes
        BOOST_CHECK_EQUAL(_mesh->GetPoint(p_id)[0], _itk->GetPoint(p_id)[0]);
        BOOST_CHECK_EQUAL(_mesh->GetPoint(p_id)[1], _itk->GetPoint(p_id)[1]);
        BOOST_CHECK_EQUAL(_mesh->GetPoint(p_id)[2], _itk->GetPoint(p_id)[2]);

        /* Uncomment below to see output of the test point set from _mesh*/
        //std::cerr << _mesh->GetPoint(p_id)[0] << " | "
        //         << _mesh->GetPoint(p_id)[1] << " | "
        //          << _mesh->GetPoint(p_id)[2] << " | " << std::endl;
    }


    // Normals //

    //std::cerr << "Normals:" << std::endl;
    for ( VC_PointsInMeshIterator point = _mesh->GetPoints()->Begin(); point != _mesh->GetPoints()->End(); ++point ) {
        // get the point's normal

        VC_PixelType _meshNormal, _itkNormal;
        _mesh->GetPointData(point.Index(), &_meshNormal);
        _itk->GetPointData(point.Index(), &_itkNormal);

        double ptNorm[3] = {_meshNormal[0], _meshNormal[1], _meshNormal[2]};
        double _itkPtNorm[3] = {_itkNormal[0], _itkNormal[1], _itkNormal[2]};

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(_meshNormal[0], _itkNormal[0]);
        BOOST_CHECK_EQUAL(_meshNormal[1], _itkNormal[1]);
        BOOST_CHECK_EQUAL(_meshNormal[2], _itkNormal[2]);

        //std::cerr << ptNorm[0] << " | " << ptNorm[1]  <<  " | " << ptNorm[2] << std::endl;
    }


    //Cells //

    // Initialize Cell Iterators
    VC_CellIterator _itkCell = _itk->GetCells()->Begin();
    VC_CellIterator _meshCell = _mesh->GetCells()->Begin();


    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(_mesh->GetNumberOfCells(), _itk->GetNumberOfCells());

    //While we have unvisited cells in the mesh
    while (_meshCell != _mesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator point = _meshCell.Value()->PointIdsBegin();
        VC_PointsInCellIterator _itkPoint = _itkCell.Value()->PointIdsBegin();

        //while we have points in the cell
        while ( point != _meshCell.Value()->PointIdsEnd() ) {


            //Now to check the points within the cells
            BOOST_CHECK_EQUAL(*point, *_itkPoint);

            //increment points
            point++;
            _itkPoint++;

        }

        //increment cells
        ++_meshCell;
        ++_itkCell;

    }

}

//
// check successful conversion from vtk mesh to itk mesh
//
// Note: _itkMesh is an itk mesh created in ivFix
//       _vtk is a vtk mesh created in ivFix
//


BOOST_FIXTURE_TEST_CASE(v2i, viFix){

    // Get smartpointer data into vtkPolyData*
    vtkPolyData* vtkRead = vtk.GetPointer();

    // initialize a new vtkPolydata pointer
    // this will hold the values for the new vtk mesh that
    // is to be compared with _vtk created in test fixture
    // NOTE: shouldn't have to use smart pointer here

    vtkPolyData* _newVtk = vtkPolyData::New();

    // convert from itk mesh to vtk and back again
    // GetPointer() from smart pointer, vtk, to pass
    volcart::meshing::vtk2itk(vtkRead, itkMesh);
    volcart::meshing::itk2vtk(itkMesh, _newVtk);

    // points + normals
    vtkDataArray *originalNormals = vtkRead->GetPointData()->GetNormals();
    vtkDataArray *_newVtkNormals = _newVtk->GetPointData()->GetNormals();

    // Points
    //std::cerr << "Points:" << std::endl;
    for ( vtkIdType p_id = 0; p_id < vtkRead->GetNumberOfPoints(); ++p_id) {

        //check the points in both vtk meshes
        BOOST_CHECK_EQUAL(vtkRead->GetPoint(p_id)[0], _newVtk->GetPoint(p_id)[0]);
        BOOST_CHECK_EQUAL(vtkRead->GetPoint(p_id)[1], _newVtk->GetPoint(p_id)[1]);
        BOOST_CHECK_EQUAL(vtkRead->GetPoint(p_id)[2], _newVtk->GetPoint(p_id)[2]);

        /* Uncomment below to see output of a test point from vtkRead */
        //std::cerr << vtkRead->GetPoint(p_id)[0] << " | "
        //          << vtkRead->GetPoint(p_id)[1] << " | "
        //          << vtkRead->GetPoint(p_id)[2] << " | " << std::endl;
    }

    //Normals
    //std::cerr << "Normals:" << std::endl;
    for (vtkIdType n_id = 0; n_id < vtkRead->GetNumberOfPoints(); n_id++){

        //Check normals (nx,ny,nz)
        BOOST_CHECK_EQUAL(originalNormals->GetTuple(n_id)[0], _newVtkNormals->GetTuple(n_id)[0]);
        BOOST_CHECK_EQUAL(originalNormals->GetTuple(n_id)[1], _newVtkNormals->GetTuple(n_id)[1]);
        BOOST_CHECK_EQUAL(originalNormals->GetTuple(n_id)[2], _newVtkNormals->GetTuple(n_id)[2]);

        /* Uncomment below to see output of a test normal from vtkRead */
        //std::cerr << originalNormals->GetTuple(n_id)[0] << " | "
        //          << originalNormals->GetTuple(n_id)[1] << " | "
        //          << originalNormals->GetTuple(n_id)[2] << " | " << std::endl;
    }


    //Cells
    //std::cerr << "Cells:" << std::endl;
    for ( vtkIdType c_id = 0; c_id < vtkRead->GetNumberOfCells(); c_id++){

        vtkCell *inputCell = vtkRead->GetCell(c_id);
        vtkCell *_newVtkCell = _newVtk->GetCell(c_id);

        //Check that points in each cell are equal
        //We have three checks since there are three points that make up a cell
        BOOST_CHECK_EQUAL(inputCell->GetPointIds()->GetId(0), _newVtkCell->GetPointIds()->GetId(0));
        BOOST_CHECK_EQUAL(inputCell->GetPointIds()->GetId(1), _newVtkCell->GetPointIds()->GetId(1));
        BOOST_CHECK_EQUAL(inputCell->GetPointIds()->GetId(2), _newVtkCell->GetPointIds()->GetId(2));


        /* Uncomment below to see output of a test normal from vtkRead */
        //std::cerr << inputCell->GetPointIds()->GetId(0) << " | "
        //          << inputCell->GetPointIds()->GetId(1) << " | "
        //          << inputCell->GetPointIds()->GetId(2) << " | " << std::endl;
    }
}

/*
 * Goal of this test case is to call vtk2itk() and compare to the saved itk.obj
 * file created by itk2vtkExample.cpp.
 */

BOOST_FIXTURE_TEST_CASE(compareSavedITK, viFix){

    // Get smartpointer data into vtkPolyData* object
    // vtkRead will be converted to itk below
    vtkPolyData* vtkRead = vtk.GetPointer();

    // convert from vtk mesh to itk mesh
    volcart::meshing::vtk2itk(vtkRead, itkMesh);

    //itkMesh now contains the data that will be compared to the
    //itk.obj file saved in testing/test_data

    //First, read the .obj file
    typedef itk::MeshFileReader< VC_MeshType > ReaderType; 
    ReaderType::Pointer reader = ReaderType::New(); 
    reader->SetFileName("itk.obj");
    reader->Update();
    
    VC_MeshType::Pointer savedITKMesh = reader->GetOutput();

    /* 
     * Now the comparisons between the test-created-itk mesh (itkMesh)
     * and the saved itk data from itk.obj (stored in savedITKMesh)
     * will occur. First, points then normals then faces. 
     */

    //Check number of points in each mesh
    BOOST_CHECK_EQUAL( itkMesh->GetNumberOfPoints(), savedITKMesh->GetNumberOfPoints() );

    std::cerr << "Compraring points..." << std::endl;
    //Now iterate over point sets and compare x/y/z values
    for ( size_t p_id = 0; p_id < itkMesh ->GetNumberOfPoints(); ++p_id) {
        
        BOOST_CHECK_EQUAL( itkMesh->GetPoint(p_id)[0], savedITKMesh->GetPoint(p_id)[0] );
        BOOST_CHECK_EQUAL( itkMesh->GetPoint(p_id)[1], savedITKMesh->GetPoint(p_id)[1] );
        BOOST_CHECK_EQUAL( itkMesh->GetPoint(p_id)[2], savedITKMesh->GetPoint(p_id)[2] );
    }

    
    // Normals //
    std::cerr << "Compraring normals..." << std::endl;

    for ( VC_PointsInMeshIterator point = itkMesh->GetPoints()->Begin(); point != itkMesh->GetPoints()->End(); ++point ) {

        VC_PixelType itkMeshNormal, savedITKNormal;
        itkMesh->GetPointData(point.Index(), &itkMeshNormal);
        savedITKMesh->GetPointData(point.Index(), &savedITKNormal);

        double ptNorm[3] = {itkMeshNormal[0], itkMeshNormal[1], itkMeshNormal[2]};
        double _itkPtNorm[3] = {savedITKNormal[0], savedITKNormal[1], savedITKNormal[2]};

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(itkMeshNormal[0], savedITKNormal[0]);
        BOOST_CHECK_EQUAL(itkMeshNormal[1], savedITKNormal[1]);
        BOOST_CHECK_EQUAL(itkMeshNormal[2], savedITKNormal[2]);

    }
    
    //Cells
    std::cerr << "Compraring cells..." << std::endl;

    // Initialize Cell Iterators
    VC_CellIterator itkMeshCell = itkMesh->GetCells()->Begin();
    VC_CellIterator savedITKCell = savedITKMesh->GetCells()->Begin();

    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(itkMesh->GetNumberOfCells(), savedITKMesh->GetNumberOfCells());

    while (itkMeshCell != itkMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator itkMeshPoint = itkMeshCell.Value()->PointIdsBegin();
        VC_PointsInCellIterator savedITKPoint = savedITKCell.Value()->PointIdsBegin();

        //while we have points in the cell
        while ( itkMeshPoint != itkMeshCell.Value()->PointIdsEnd() ) {


            //Now to check the points within the cells
            BOOST_CHECK_EQUAL(*itkMeshPoint, *savedITKPoint);

            //increment points
            itkMeshPoint++;
            savedITKPoint++;

        }

        //increment cells
        ++itkMeshCell;
        ++savedITKCell;

    }

}

/*
 * Goal of this test case is to call itk2vtk() and compare to the saved vtk.ply
 * file created by itk2vtkExample.cpp.
 */

BOOST_FIXTURE_TEST_CASE(compareSavedVTK, ivFix){

    // convert from itk mesh to vtk and back again
    volcart::meshing::itk2vtk(_mesh, _vtk);

    //Init two vectors to hold the saved vtk file information
    std::vector<VC_Vertex> savedVTKPoints;
    std::vector<VC_Cell> savedVTKCells;

    //Read in the vtk.ply file
    //parsePlyFile() found in parsingHelpers.cpp
    parsePlyFile("vtk.ply", savedVTKPoints, savedVTKCells);

    /*
     * Now the comparison can occur between the saved .ply file and the
     * test-case-created _vtk mesh. First, we check the points and then
     * the cells.
     */

    //Confirm equal number of points in both meshes
    BOOST_CHECK_EQUAL(savedVTKPoints.size(), _vtk->GetNumberOfPoints());

    //Check equivalency of points
    for ( size_t p_id = 0; p_id < _vtk->GetNumberOfPoints(); ++p_id) {

        //check the points in both vtk meshes
        BOOST_CHECK_EQUAL(savedVTKPoints[p_id].x, _vtk->GetPoint(p_id)[0]);
        BOOST_CHECK_EQUAL(savedVTKPoints[p_id].y, _vtk->GetPoint(p_id)[1]);
        BOOST_CHECK_EQUAL(savedVTKPoints[p_id].z, _vtk->GetPoint(p_id)[2]);
    }

    //Cells (faces)

    //Confirm same number of faces in saved VTK file versus converted VTK
    BOOST_CHECK_EQUAL(savedVTKCells.size(), _vtk->GetNumberOfCells());

    for ( vtkIdType c_id = 0; c_id < _vtk->GetNumberOfCells(); c_id++){

        vtkCell *inputCell = _vtk->GetCell(c_id);

        //Check that vertices making up the corresponding cells are equal
        //We have three checks since there are three points that make up a cell
        BOOST_CHECK_EQUAL(inputCell->GetPointIds()->GetId(0), savedVTKCells[c_id].v1);
        BOOST_CHECK_EQUAL(inputCell->GetPointIds()->GetId(1), savedVTKCells[c_id].v2);
        BOOST_CHECK_EQUAL(inputCell->GetPointIds()->GetId(2), savedVTKCells[c_id].v3);

    }

}