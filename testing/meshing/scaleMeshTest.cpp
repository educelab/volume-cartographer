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
#include "scaleMesh.h"

/************************************************************************************
 *                                                                                  *
 *  scaleMeshTest.cpp - tests the functionality of /v-c/meshing/scaleMesh.cpp       *
 *  The ultimate goal of this file is the following:                                *
 *                                                                                  *
 *        1. check whether an itk mesh can be converted to a vtk mesh               *
 *           and vice versa.                                                        *
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