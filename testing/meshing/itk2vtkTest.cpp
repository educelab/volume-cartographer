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
 *  This file is broken up into a testing fixture, ivFix, which initializes the     *
 *  objects used in each of the two test cases.                                     *
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
 * This builds objects for the test cases below that reference
 * the fixture as their second argument
 *
 */

struct ivFix {

    ivFix() {

        //need to create a pointer to itk and vtk meshes
        _mesh = mesh.itkMesh();
      //  _vtk = vtkPolyData::New();
        std::cerr << "setting up itk2vtkTest objects" << std::endl;
    }

    ~ivFix(){ std::cerr << "cleaning up itk2vtkTest objects" << std::endl; }

    VC_MeshType::Pointer _mesh ;
    volcart::testing::testingMesh mesh;
    //vtkPolyData * _vtk;
};

//
// check successful conversion from itk mesh to vtk mesh
//

BOOST_FIXTURE_TEST_CASE(i2v, ivFix){

    // initialize a new VC_MeshType::Pointer
    // this will hold the values for the new itk mesh that
    // is to be compared with _mesh.
    VC_MeshType::Pointer _itk = VC_MeshType::New();

    vtkPolyData* _vtk = vtkPolyData::New();

    // convert from itk mesh to vtk and back again
    volcart::meshing::itk2vtk(_mesh, _vtk);
    volcart::meshing::vtk2itk(_vtk, _itk);

    VC_PointsContainerType::Pointer _meshPoints = _mesh->GetPoints();
    VC_PointsContainerType::Pointer convertedPoints = _itk->GetPoints();


    // Initialize iterators
    VC_CellIterator  _meshCellIterator = _mesh->GetCells()->Begin();
    VC_CellIterator  _meshCellEnd      = _mesh->GetCells()->End();

    VC_CellIterator  convertedCellIterator = _itk->GetCells()->Begin();
    VC_CellIterator  convertedCellEnd      = _itk->GetCells()->End();

    VC_CellType * _meshCell;
    VC_CellType * convertedCell;

    VC_PointsInCellIterator _meshPointsIterator;

    VC_PointType _meshP;
    VC_PointType convertedP;

    unsigned long _meshPointID, convertedPointID;

    // Iterate over all of the cells in the mesh to compare values
    while( ( _meshCellIterator != _meshCellEnd ) &&
                    ( convertedCellIterator != convertedCellEnd) )
    {
        // Link the pointer to our current cell
        _meshCell = _meshCellIterator.Value();
        convertedCell = convertedCellIterator.Value();

        // Iterate over the vertices of the current cell for both meshes
        _meshPointsIterator = _meshCell->PointIdsBegin();

        while( _meshPointsIterator != _meshCell->PointIdsEnd() ) {

            _meshPointID = *_meshPointsIterator;

            BOOST_CHECK_EQUAL(_mesh->GetPoint(_meshPointID), _itk->GetPoint(_meshPointID));

            ++_meshPointsIterator;
        }
        
        ++_meshCellIterator;
        ++convertedCellIterator;
    }


}


// check successful conversion from itk mesh to vtk mesh
BOOST_FIXTURE_TEST_CASE(v2i, ivFix){



    BOOST_CHECK(true);

}
