//
// Created by Ryan Taber on 1/29/16.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE scaleMesh

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include <io/objWriter.h>
#include "vc_defines.h"
#include "shapes.h"
#include "scaleMesh.h"

/************************************************************************************
 *                                                                                  *
 *  scaleMeshTest.cpp - tests the functionality of /v-c/meshing/scaleMesh.cpp       *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether an itk mesh can be scaled correctly without changing     *
 *           the input mesh.                                                        *
 *                                                                                  *
 *  This file is broken up into test fixtures for each of the common mesh shapes    *
 *  (plane, cube, cone, sphere, arch) which initialize objects for the following    *
 *  test cases:                                                                     *
 *                                                                                  *
 *  1. planeTest (planeFix)                                                         *
 *  2. cubeTest (cubeFix)                                                           *
 *  3. archTest (archFix)                                                           *
 *  4. sphereTest (sphereFix)                                                       *
 *  5. coneTest (coneFix)                                                           *
 *  6. inputUnchangedTest (planeFix)                                                *   
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
 * Individual test fixtures for each of the common shapes
 * Test cases call appropriate fixture
 */

struct planeFix {

    planeFix() {

        inputMesh = plane.itkMesh();

        std::cerr << "setting up planar mesh for scaling" << std::endl;
    }

    ~planeFix(){ std::cerr << "cleaning up plane objects" << std::endl; }

    VC_MeshType::Pointer inputMesh;
    VC_MeshType::Pointer outputMesh = VC_MeshType::New();
    volcart::shapes::Plane plane;
};

struct cubeFix {

    cubeFix() {

        inputMesh = cube.itkMesh();

        std::cerr << "setting up cube mesh for scaling" << std::endl;
    }

    ~cubeFix(){ std::cerr << "cleaning up cube objects" << std::endl; }

    VC_MeshType::Pointer inputMesh;
    VC_MeshType::Pointer outputMesh = VC_MeshType::New();
    volcart::shapes::Cube cube;
};

struct archFix {

    archFix() {

        inputMesh = arch.itkMesh();

        std::cerr << "setting up arch mesh for scaling" << std::endl;
    }

    ~archFix(){ std::cerr << "cleaning up arch mesh objects" << std::endl; }

    VC_MeshType::Pointer inputMesh;
    VC_MeshType::Pointer outputMesh = VC_MeshType::New();
    volcart::shapes::Arch arch;
};

struct sphereFix {

    sphereFix() {

        inputMesh = sphere.itkMesh();

        std::cerr << "setting up spherical mesh for scaling" << std::endl;
    }

    ~sphereFix(){ std::cerr << "cleaning up spherical mesh for scaling" << std::endl; }

    VC_MeshType::Pointer inputMesh;
    VC_MeshType::Pointer outputMesh = VC_MeshType::New();
    volcart::shapes::Sphere sphere;
};

struct coneFix {

    coneFix() {

        inputMesh = cone.itkMesh();

        std::cerr << "setting up cone mesh for scaling" << std::endl;
    }

    ~coneFix(){ std::cerr << "cleaning up cone mesh for scaling" << std::endl; }

    VC_MeshType::Pointer inputMesh;
    VC_MeshType::Pointer outputMesh = VC_MeshType::New();
    volcart::shapes::Cone cone;
};


BOOST_FIXTURE_TEST_CASE(planeTest, planeFix){

    //scaling factor <,>,== 0 all used here
    double factor = 3;

    //loop to cover positive, negative and zero value of factor
    while (factor > -1.5){

        //call scaleMesh        TODO: outputMesh coming back NULL from scaleMesh()
        volcart::meshing::scaleMesh(inputMesh, outputMesh, factor);

        BOOST_CHECK_EQUAL(inputMesh->GetNumberOfPoints(), outputMesh->GetNumberOfPoints());


        for (size_t p; p < outputMesh->GetNumberOfPoints(); ++p){

            //check each of the points in the input and output meshes to confirm
            //ratio matches scale factor

            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[0] * factor, outputMesh->GetPoint(p)[0]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[1] * factor, outputMesh->GetPoint(p)[1]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[2] * factor, outputMesh->GetPoint(p)[2]);

        }

        factor -= 0.5;
    }

}

BOOST_FIXTURE_TEST_CASE(cubeTest, cubeFix){

    double factor = 3;

    while (factor > -1.5){

        volcart::meshing::scaleMesh(inputMesh, outputMesh, factor);

        BOOST_CHECK_EQUAL(inputMesh->GetNumberOfPoints(), outputMesh->GetNumberOfPoints());


        for (size_t p; p < outputMesh->GetNumberOfPoints(); ++p){

            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[0] * factor, outputMesh->GetPoint(p)[0]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[1] * factor, outputMesh->GetPoint(p)[1]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[2] * factor, outputMesh->GetPoint(p)[2]);
        }

        factor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(archTest, archFix){

    double factor = 3;

    while (factor > -1.5){

        volcart::meshing::scaleMesh(inputMesh, outputMesh, factor);

        volcart::io::objWriter writer;

        BOOST_CHECK_EQUAL(inputMesh->GetNumberOfPoints(), outputMesh->GetNumberOfPoints());

        for (size_t p; p < outputMesh->GetNumberOfPoints(); ++p){

            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[0] * factor, outputMesh->GetPoint(p)[0]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[1] * factor, outputMesh->GetPoint(p)[1]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[2] * factor, outputMesh->GetPoint(p)[2]);
        }

        factor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(sphereTest, sphereFix){

    double factor = 3;

    while (factor > -1.5){

        volcart::meshing::scaleMesh(inputMesh, outputMesh, factor);

        BOOST_CHECK_EQUAL(inputMesh->GetNumberOfPoints(), outputMesh->GetNumberOfPoints());

        for (size_t p; p < outputMesh->GetNumberOfPoints(); ++p){

            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[0] * factor, outputMesh->GetPoint(p)[0]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[1] * factor, outputMesh->GetPoint(p)[1]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[2] * factor, outputMesh->GetPoint(p)[2]);
        }

        factor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(coneTest, coneFix){

    double factor = 3;

    while (factor > -1.5){

        volcart::meshing::scaleMesh(inputMesh, outputMesh, factor);

        BOOST_CHECK_EQUAL(inputMesh->GetNumberOfPoints(), outputMesh->GetNumberOfPoints());

        for (size_t p; p < outputMesh->GetNumberOfPoints(); ++p){

            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[0] * factor, outputMesh->GetPoint(p)[0]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[1] * factor, outputMesh->GetPoint(p)[1]);
            BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[2] * factor, outputMesh->GetPoint(p)[2]);
        }

        factor -= 0.5;
    }
}

BOOST_FIXTURE_TEST_CASE(inputUnchangedTest, planeFix){

    //scaling factor arbitrary
    double factor = 25;

    //call scaleMesh
    volcart::meshing::scaleMesh(inputMesh, outputMesh, factor);

    //init new plane mesh
    VC_MeshType::Pointer planeMesh = plane.itkMesh();

    //compare inputMesh and planeMesh to confirm inputMesh goes through
    //scaleMesh unchanged

    BOOST_CHECK_EQUAL(inputMesh->GetNumberOfPoints(), planeMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(inputMesh->GetNumberOfCells(), planeMesh->GetNumberOfCells());

    for (size_t p; p < inputMesh->GetNumberOfPoints(); ++p){

        BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[0], planeMesh->GetPoint(p)[0]);
        BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[1], planeMesh->GetPoint(p)[1]);
        BOOST_CHECK_EQUAL(inputMesh->GetPoint(p)[2], planeMesh->GetPoint(p)[2]);

    }

    // Compare Cells
    VC_CellIterator inputMeshCell = inputMesh->GetCells()->Begin();
    VC_CellIterator planeMeshCell = planeMesh->GetCells()->Begin();

    int c = 0;
    while (inputMeshCell != inputMesh->GetCells()->End()) {

        //Initialize Iterators for Points in a Cell
        VC_PointsInCellIterator inputMeshPoint = inputMeshCell.Value()->PointIdsBegin();
        VC_PointsInCellIterator planeMeshPoint = planeMeshCell.Value()->PointIdsBegin();

        int counter = 0;
        //while we have points in the cell
        while ( inputMeshPoint != inputMeshCell.Value()->PointIdsEnd() ) {

            //Now to check the points within the cells
            if (counter == 0)
                BOOST_CHECK_EQUAL(*inputMeshPoint, *planeMeshPoint);
            else if(counter == 1)
                BOOST_CHECK_EQUAL(*inputMeshPoint, *planeMeshPoint);
            else if (counter == 2)
                BOOST_CHECK_EQUAL(*inputMeshPoint, *planeMeshPoint);

            //increment points
            ++inputMeshPoint; ++planeMeshPoint; ++counter;

        }

        //increment cells
        ++inputMeshCell; ++planeMeshCell;
    }

}
