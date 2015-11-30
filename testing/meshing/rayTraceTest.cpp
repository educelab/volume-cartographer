//
// Created by Ryan Taber on 11/30/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE rayTrace


#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>

#include "rayTrace.h"

/************************************************************************************
 *                                                                                  *
 *  orderedPCDMesherTest.cpp - tests the functionality of                           *
 *  v-c/meshing/orderedPCDMesher.cpp with the ultimate goal of the following:       *
 *                                                                                  *
 *     Given the same input point cloud, does a saved PLY file match a current      *
 *     execution of orederedPCDMesher().                                            *
 *                                                                                  *
 *  This file is broken up into a test fixture orderedPCDFix which initialize       *
 *  the objects used in any subsequent fixture test cases.                          *
 *                                                                                  *
 *  Test Cases:                                                                     *
 *  1. orderedPCDTest (fixture test case)                                           *
 *                                                                                  *                                                                            *
 *  Input:                                                                          *
 *     No required inputs for the test cases. Any test objects are created          *
 *     internally by orderedPCDFix() or within the test cases themselves.           *
 *                                                                                  *
 *  Test-Specific Output:                                                           *
 *     Specific test output only given on failure of any tests. Otherwise, general  *
 *     number of testing errors is output to console.                               *
 *                                                                                  *
 *  Miscellaneous:                                                                  *
 *     See the /testing/meshing wiki for more information on this test              *
 * **********************************************************************************/

/*
 * Purpose of orderedPCDFix:
 *      - generate a point cloud consisting of PointXYZRGB points
 *      - call orderedPCDMesher() on this point cloud and write to file
 */

struct rayTraceFix {

    rayTraceFix() {

        //generate the curved mesh
        _mesh = iMesh.itkMesh();

        //call orderedPCD()
        volcart::meshing::orderedPCDMesher(cloud, outfile);

        std::cerr << "\nsetting up rayTraceTest objects" << std::endl;
    }

    ~rayTraceFix(){ std::cerr << "\ncleaning up rayTraceTest objects" << std::endl; }

    pcl::PointCloud<pcl::PointXYZRGB> pCloud;
    volcart::shapes::Plane mesh;
    std::string outfile;


        std::vector<cv::Vec6f> traceResults;

        volcart::meshing::rayTrace(itkMesh,  traceDir, width, height, std::map<int, cv::Vec2d> &uvMap) {

        // Essential data structure to return points and normals
        std::vector<cv::Vec6f> intersections;
        VC_MeshType::Pointer iMesh;
        volcart::shapes::Plane mesh;
        std::map<int, cv::Vec2d> &uvMap;
        int traceDir = 0; //default direction is anything != 1
        int width, height;
        
        // Convert the itk mesh to a vtk mesh
        vtkPolyData *vtkMesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(itkMesh, vtkMesh);




    };

/*
 * Test to see that a saved PLY file from fixture matches a recalled orderedPCDMesher()
 * using the same input point cloud.
 */
BOOST_FIXTURE_TEST_CASE(rayTest, rayTraceFix){


}
