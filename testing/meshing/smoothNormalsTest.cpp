//
// Created by Ryan Taber on 1/29/16.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE smoothNormals

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "shapes.h"
#include "parsingHelpers.h"
#include "smoothNormals.h"


/************************************************************************************
 *                                                                                  *
 *  smoothNormalsTest.cpp - tests the functionality of meshing/smoothNormals.cpp    *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. confirm volcart::meshing::smoothNormals() works as expected            *
 *                                                                                  *
 *  This file is broken up into a test fixture (normalFix) which initializes        *
 *  the objects used for the test case.                                             *
 *                                                                                  *
 *  1. compareSmoothedPlane (fixture test case)                                     *
 *  2. compareSmoothedCube (fixture test case)                                      *
 *  3. compareSmoothedSphere (fixture test case)                                    *
 *  4. compareSmoothedArch (fixture test case)                                      *
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
 * This builds objects for the case below that reference
 * the fixture as their second argument
 *
 */

struct normalFix {

    normalFix() {

        planeMesh = plane.itkMesh();
        cubeMesh = cube.itkMesh();
        archMesh = arch.itkMesh();
        sphereMesh = sphere.itkMesh();
        savedFactor = 4;
        tolerance = 0.00001; //TODO: confirm small enough value

        std::cerr << "setting up smoothNormals objects" << std::endl;
    }

    ~normalFix(){ std::cerr << "cleaning up smoothNormals objects" << std::endl; }

    VC_MeshType::Pointer planeMesh, cubeMesh, sphereMesh, archMesh, smoothedMesh;
    volcart::shapes::Plane plane;
    volcart::shapes::Cube cube;
    volcart::shapes::Sphere sphere;
    volcart::shapes::Arch arch;
    double tolerance;   //used to check equivalency of values from saved files
    double savedFactor;

};


/*
 * The next four tests use the obj files representing the smoothed shapes created by smoothingExample.cpp
 * and compares these files to test-specific calls smoothNormals() using the same shape objects.
 *
 * Smoothing factor should be 4 for each test case
 *
 * Split the tests into four cases for log purposes and pinpointing errors faster if there should be
 * an issue in the future.
 *
 */

BOOST_FIXTURE_TEST_CASE(compareSmoothedPlane, normalFix){

    //call smoothNormals fixture-generated plane
    smoothedMesh = volcart::meshing::smoothNormals(planeMesh, savedFactor);

    //init vectors to hold points and cells from savedITK data file
    std::vector<VC_Vertex> savedITKPoints;
    std::vector<VC_Cell> savedITKCells;

    //First, read the saved .obj file (cmake should copy from test_data to current build dir)
    volcart::testing::ParsingHelpers::parseObjFile("smoothedPlane.obj", savedITKPoints, savedITKCells);

    /*
     * Now the comparisons between the test-created-itk mesh (smoothedMesh)
     * and the saved data from smoothedPlane.obj (stored in savedITKPoints and savedITKCells)
     * will occur. Points --> normals --> faces.
     */

    //Check number of points in each mesh
    BOOST_CHECK_EQUAL( smoothedMesh->GetNumberOfPoints(), savedITKPoints.size() );

    std::cerr << "Comparing points..." << std::endl;
    //Now iterate over point sets and compare x/y/z values
    for ( size_t p_id = 0; p_id < smoothedMesh ->GetNumberOfPoints(); ++p_id) {


        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[0], savedITKPoints[p_id].x, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[1], savedITKPoints[p_id].y, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[2], savedITKPoints[p_id].z, tolerance);
    }

    std::cerr << "Comparing normals..." << std::endl;
    int p = 0;
    for ( VC_PointsInMeshIterator point = smoothedMesh->GetPoints()->Begin(); point != smoothedMesh->GetPoints()->End(); ++point ) {

        VC_PixelType Normal;
        smoothedMesh->GetPointData(point.Index(), &Normal);

        //double ptNorm[3] = {planeNormal[0], planeNormal[1], planeNormal[2]};

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(Normal[0], savedITKPoints[p].nx);
        BOOST_CHECK_EQUAL(Normal[1], savedITKPoints[p].ny);
        BOOST_CHECK_EQUAL(Normal[2], savedITKPoints[p].nz);

        p++;

    }

    std::cerr << "Comparing faces..." << std::endl;

    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(smoothedMesh->GetNumberOfCells(), savedITKCells.size());

    // Initialize Cell Iterators
    VC_CellIterator planeCell = smoothedMesh->GetCells()->Begin();

    int c = 0;

    while (planeCell != smoothedMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator planeMeshPoint = planeCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( planeMeshPoint != planeCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*planeMeshPoint, savedITKCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*planeMeshPoint, savedITKCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*planeMeshPoint, savedITKCells[c].v3);

            //increment points
            planeMeshPoint++;
            counter++;

        }

        //increment cells
        ++planeCell;
        ++c;
    }
}


BOOST_FIXTURE_TEST_CASE(compareSmoothedCube, normalFix){

    //call smoothNormals fixture-generated plane
    smoothedMesh = volcart::meshing::smoothNormals(cubeMesh, savedFactor);

    //init vectors to hold points and cells from savedITK data file
    std::vector<VC_Vertex> savedITKPoints;
    std::vector<VC_Cell> savedITKCells;

    //First, read the saved .obj file (cmake should copy from test_data to current build dir
    volcart::testing::ParsingHelpers::parseObjFile("smoothedCube.obj", savedITKPoints, savedITKCells);

    /*
     * Now the comparisons between the test-created-itk mesh (smoothedMesh)
     * and the saved data from smoothedPlane.obj (stored in savedITKPoints and savedITKCells)
     * will occur. Points --> normals --> faces.
     */

    //Check number of points in each mesh
    BOOST_CHECK_EQUAL( smoothedMesh->GetNumberOfPoints(), savedITKPoints.size() );

    std::cerr << "Comparing points..." << std::endl;
    //Now iterate over point sets and compare x/y/z values
    for ( size_t p_id = 0; p_id < smoothedMesh ->GetNumberOfPoints(); ++p_id) {

        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[0], savedITKPoints[p_id].x, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[1], savedITKPoints[p_id].y, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[2], savedITKPoints[p_id].z, tolerance);
    }

    std::cerr << "Comparing normals..." << std::endl;
    int p = 0;
    for ( VC_PointsInMeshIterator point = smoothedMesh->GetPoints()->Begin(); point != smoothedMesh->GetPoints()->End(); ++point ) {

        VC_PixelType Normal;
        smoothedMesh->GetPointData(point.Index(), &Normal);

        //double ptNorm[3] = {planeNormal[0], planeNormal[1], planeNormal[2]};

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(Normal[0], savedITKPoints[p].nx);
        BOOST_CHECK_EQUAL(Normal[1], savedITKPoints[p].ny);
        BOOST_CHECK_EQUAL(Normal[2], savedITKPoints[p].nz);

        p++;

    }

    std::cerr << "Comparing faces..." << std::endl;

    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(smoothedMesh->GetNumberOfCells(), savedITKCells.size());

    // Initialize Cell Iterators
    VC_CellIterator cubeCell = smoothedMesh->GetCells()->Begin();

    int c = 0;

    while (cubeCell != smoothedMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator cubeMeshPoint = cubeCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( cubeMeshPoint != cubeCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*cubeMeshPoint, savedITKCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*cubeMeshPoint, savedITKCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*cubeMeshPoint, savedITKCells[c].v3);

            //increment points
            cubeMeshPoint++;
            counter++;

        }

        //increment cells
        ++cubeCell;
        ++c;
    }
}

BOOST_FIXTURE_TEST_CASE(compareSmoothedSphere, normalFix){

    //call smoothNormals fixture-generated plane
    smoothedMesh = volcart::meshing::smoothNormals(sphereMesh, savedFactor);

    //init vectors to hold points and cells from savedITK data file
    std::vector<VC_Vertex> savedITKPoints;
    std::vector<VC_Cell> savedITKCells;

    //First, read the saved .obj file (cmake should copy from test_data to current build dir
    volcart::testing::ParsingHelpers::parseObjFile("smoothedSphere.obj", savedITKPoints, savedITKCells);

    /*
     * Now the comparisons between the test-created-itk mesh (smoothedMesh)
     * and the saved data from smoothedPlane.obj (stored in savedITKPoints and savedITKCells)
     * will occur. Points --> normals --> faces.
     */

    //Check number of points in each mesh
    BOOST_CHECK_EQUAL( smoothedMesh->GetNumberOfPoints(), savedITKPoints.size() );

    std::cerr << "Comparing points..." << std::endl;
    //Now iterate over point sets and compare x/y/z values
    for ( size_t p_id = 0; p_id < smoothedMesh ->GetNumberOfPoints(); ++p_id) {

        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[0], savedITKPoints[p_id].x, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[1], savedITKPoints[p_id].y, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[2], savedITKPoints[p_id].z, tolerance);
    }

    std::cerr << "Comparing normals..." << std::endl;
    int p = 0;
    for ( VC_PointsInMeshIterator point = smoothedMesh->GetPoints()->Begin(); point != smoothedMesh->GetPoints()->End(); ++point ) {

        VC_PixelType Normal;
        smoothedMesh->GetPointData(point.Index(), &Normal);

        //double ptNorm[3] = {planeNormal[0], planeNormal[1], planeNormal[2]};

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(Normal[0], savedITKPoints[p].nx);
        BOOST_CHECK_EQUAL(Normal[1], savedITKPoints[p].ny);
        BOOST_CHECK_EQUAL(Normal[2], savedITKPoints[p].nz);

        p++;

    }

    std::cerr << "Comparing faces..." << std::endl;

    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(smoothedMesh->GetNumberOfCells(), savedITKCells.size());

    // Initialize Cell Iterators
    VC_CellIterator sphereCell = smoothedMesh->GetCells()->Begin();

    int c = 0;

    while (sphereCell != smoothedMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator sphereMeshPoint = sphereCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( sphereMeshPoint != sphereCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*sphereMeshPoint, savedITKCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*sphereMeshPoint, savedITKCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*sphereMeshPoint, savedITKCells[c].v3);

            //increment points
            sphereMeshPoint++;
            counter++;

        }

        //increment cells
        ++sphereCell;
        ++c;
    }
}

BOOST_FIXTURE_TEST_CASE(compareSmoothedArch, normalFix){

    //call smoothNormals fixture-generated plane
    smoothedMesh = volcart::meshing::smoothNormals(archMesh, savedFactor);

    //init vectors to hold points and cells from savedITK data file
    std::vector<VC_Vertex> savedITKPoints;
    std::vector<VC_Cell> savedITKCells;

    //First, read the saved .obj file (cmake should copy from test_data to current build dir
    volcart::testing::ParsingHelpers::parseObjFile("smoothedArch.obj", savedITKPoints, savedITKCells);

    /*
     * Now the comparisons between the test-created-itk mesh (smoothedMesh)
     * and the saved data from smoothedPlane.obj (stored in savedITKPoints and savedITKCells)
     * will occur. Points --> normals --> faces.
     */

    //Check number of points in each mesh
    BOOST_CHECK_EQUAL( smoothedMesh->GetNumberOfPoints(), savedITKPoints.size() );

    std::cerr << "Comparing points..." << std::endl;
    //Now iterate over point sets and compare x/y/z values
    for ( size_t p_id = 0; p_id < smoothedMesh ->GetNumberOfPoints(); ++p_id) {

        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[0], savedITKPoints[p_id].x, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[1], savedITKPoints[p_id].y, tolerance);
        BOOST_CHECK_CLOSE(smoothedMesh->GetPoint(p_id)[2], savedITKPoints[p_id].z, tolerance);
    }

    std::cerr << "Comparing normals..." << std::endl;
    int p = 0;
    for ( VC_PointsInMeshIterator point = smoothedMesh->GetPoints()->Begin(); point != smoothedMesh->GetPoints()->End(); ++point ) {

        VC_PixelType Normal;
        smoothedMesh->GetPointData(point.Index(), &Normal);

        //double ptNorm[3] = {planeNormal[0], planeNormal[1], planeNormal[2]};

        //Now compare the normals for the two meshes
        BOOST_CHECK_EQUAL(Normal[0], savedITKPoints[p].nx);
        BOOST_CHECK_EQUAL(Normal[1], savedITKPoints[p].ny);
        BOOST_CHECK_EQUAL(Normal[2], savedITKPoints[p].nz);

        p++;

    }

    std::cerr << "Comparing faces..." << std::endl;

    //compare number of cells in each itk mesh
    BOOST_CHECK_EQUAL(smoothedMesh->GetNumberOfCells(), savedITKCells.size());

    // Initialize Cell Iterators
    VC_CellIterator archCell = smoothedMesh->GetCells()->Begin();

    int c = 0;

    while (archCell != smoothedMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator archMeshPoint = archCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( archMeshPoint != archCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*archMeshPoint, savedITKCells[c].v1);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*archMeshPoint, savedITKCells[c].v2);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*archMeshPoint, savedITKCells[c].v3);

            //increment points
            archMeshPoint++;
            counter++;

        }

        //increment cells
        ++archCell;
        ++c;
    }
}

/*
 * Testing a zero radius smoothing factor
 * Uses plane mesh
 */

BOOST_FIXTURE_TEST_CASE(zeroRadius, normalFix){

    double radius = 5;
    smoothedMesh = volcart::meshing::smoothNormals(archMesh, radius);

    //check number of points and cells are equivalent between the two meshes
    BOOST_CHECK_EQUAL(archMesh->GetNumberOfPoints(), smoothedMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(archMesh->GetNumberOfCells(), smoothedMesh->GetNumberOfCells());

    for (size_t point = 0; point < archMesh->GetNumberOfPoints(); ++point){

        std::cout << "S Point " << point << ":"
                  << smoothedMesh->GetPoint(point)[0] << " "
                  << smoothedMesh->GetPoint(point)[1] << " "
                  << smoothedMesh->GetPoint(point)[2] << " "
                  << smoothedMesh->GetPoint(point)[3] << " "
                  << smoothedMesh->GetPoint(point)[4] << " "
                  << smoothedMesh->GetPoint(point)[5] << std::endl;

        std::cout << "A Point " << point << ":"
                  << archMesh->GetPoint(point)[0] << " "
                  << archMesh->GetPoint(point)[1] << " "
                  << archMesh->GetPoint(point)[2] << " "
                  << archMesh->GetPoint(point)[3] << " "
                  << archMesh->GetPoint(point)[4] << " "
                  << archMesh->GetPoint(point)[5] << std::endl;
    }

}