//
// Created by Ryan Taber on 10/14/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE itk2vtk

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "itk2vtk.h"


/************************************************************************************
 *                                                                                  *
 *  it2vtkTest.cpp - tests the functionality of /v-c/meshing/it2vtk.cpp             *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether an itk mesh can be converted to a vtk mesh               *
 *           and vice versa.                                                        *
 *           common/testing/testingMesh.cpp, can be written into                    *
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
    volcart::testing::testingMesh mesh;
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
    volcart::testing::testingMesh mesh;
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

    VC_PointsContainerType::Pointer _meshPoints = _mesh->GetPoints();
    VC_PointsContainerType::Pointer _itkPoints = _itk->GetPoints();


    // Initialize iterators
    VC_CellIterator  _meshCellIterator = _mesh->GetCells()->Begin();
    VC_CellIterator  _meshCellEnd      = _mesh->GetCells()->End();

    VC_CellIterator  _itkCellIterator = _itk->GetCells()->Begin();
    VC_CellIterator  _itkCellEnd = _itk->GetCells()->End();

    VC_CellType * _meshCell;
    VC_CellType * _itkCell;

    VC_PointsInCellIterator _meshPointsIterator;

    VC_PointType _meshP;
    VC_PointType _itkP;

    unsigned long _meshPointID;

    // Iterate over all of the cells in the mesh to compare values
    while( ( _meshCellIterator != _meshCellEnd ) &&
                    ( _itkCellIterator != _itkCellEnd) )
    {
        // Link the pointer to our current cell
        _meshCell = _meshCellIterator.Value();
        _itkCell = _itkCellIterator.Value();

        // Iterate over the vertices of the current cell for both meshes
        _meshPointsIterator = _meshCell->PointIdsBegin();

        // Until we reach the final point, get the point id and compare the values
        // of the points in the original (_mesh) and new (_itk) meshes. Advance iterator
        //and continue this process until all the cells in each mesh have been visited (first while loop)
        while( _meshPointsIterator != _meshCell->PointIdsEnd() ) {

            _meshPointID = *_meshPointsIterator;

            BOOST_CHECK_EQUAL(_mesh->GetPoint(_meshPointID), _itk->GetPoint(_meshPointID));

            ++_meshPointsIterator;
        }

        ++_meshCellIterator;
        ++_itkCellIterator;
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
