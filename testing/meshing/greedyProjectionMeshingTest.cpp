//
// Created by Melissa Shankle on 10/26/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE greedyProjectionMeshing

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "greedyProjectionMeshing.h"

/****************************************************************************************
 *                                                                                      *
 *  greedyProjectionMeshingTest.cpp -  tests the functionality of changes to            *
 *      /v-c/meshing/greedyProjectionMeshing.cpp                                        *
 *  The ultimate goal of this file is the following:                                    *
 *                                                                                      *
 *      1. Check whether changes to greedyProjectionMeshing.cpp create a                *
 *         correct mesh based on a previously created, correct mesh.                    *
 *                                                                                      *
 *  Input:                                                                              *
 *      - Point Cloud Pointer for the point cloud undergoing greedyProjectionMeshing    *
 *          - pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr                              *
 *      - Max number of neighbors                                                       *
 *          - unsigned                                                                  *
 *      - Radius to determine the maximum distance between connected parts              *
 *          - double                                                                    *
 *      - Radius multiplier to determine the maximum distance from the center point     *
 *          - double                                                                    *
 *      - Point Cloud Pointer to the known point cloud                                  *
 *          - pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr                              *
 *                                                                                      *
 *  Test-Specific Output:                                                               *
 *      Specific test output only given on failure of any test. Others, general number  *
 *      of testing errors is output.                                                    *
 *                                                                                      *
 *  Miscellaneous:                                                                      *
 *      See the /testing/meshing wiki for more information on this test.                *
 *                                                                                      *
 ***************************************************************************************/

// General outline for test
//   Take in input for test
//   Create new mesh using greedyProjectionMeshing
//   Compare new mesh with known mesh for equivalency
//   If errors occur, output them


/*
 * This builds objects for the test cases below that reference
 * the fixture as their second argument
 */

struct Fix {

    Fix() {

        std::cerr << "Setting up greedyProjectionMeshingTest objects." << std::endl;
    }

    ~Fix(){ std::cerr << "Cleaning up greedyProjectionMeshing objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_mesh, known_mesh;
};

BOOST_FIXTURE_TEST_CASE(new_mesh, Fix){

    // Get the points for each mesh to use for comparison
    VC_PointsContainerType::Pointer _newMeshPoints = new_mesh->GetPoints();
    VC_PointsContainerType::Pointer _knownMeshPoints = known_mesh->GetPoints();

    // Initialize iterators for comparison
    VC_CellIterator _newMeshCellIterator = new_mesh->GetCells()->Begin();
    VC_CellIterator _newMeshCellEnd new_mesh->GetCells()->End();

    VC_CellIterator _knownMeshCellIterator = known_mesh->GetCells()->Begin();
    VC_CellIterator _knownMeshCellEnd = known_mesh->GetCells()->End;

    VC_PointsInCellIterator _meshPointsIterator;

    VC_CellType * _meshCell;

    unsigned long _meshPointID;

    // Iterate over all of the cells in the mesh to compare values
    while( ( _newMeshCellIterator != _newMeshCellEnd ) &&
            ( _knownMeshCellIterator != _knownMeshCellEnd) )
    {
        //Link the pointer to our current cell
        _meshCell = _newMeshCellIterator.Value();

        // Iterate over the vertices of the current cell for both meshes
        _meshPointsIterator = _meshCell->PointIdsBegin();

        //Until we reach the final point, get the point id and compare the values
        // of the points in the original (_mesh) and new (_itk) meshes. Advance iterator
        //and continue this process until all the cells in each mesh have been visited (first while loop)
    }



