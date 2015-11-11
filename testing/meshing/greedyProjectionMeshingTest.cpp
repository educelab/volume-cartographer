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

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include "pcl/conversions.h"


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
        // Create a mesh

        pcl::PointCloud<pcl::PointNormal> cloud_PointNormal = mesh.pointCloudNormal();

        // Create pointer for the mesh to pass to greedyProjectionMeshing
        pcl::PointCloud<pcl::PointNormal>::Ptr input( new pcl::PointCloud<pcl::PointNormal>);
        *input = cloud_PointNormal;

        // Call function from namespace in header file
        std::cout << "Being greedy..." << std::endl;
        output = volcart::meshing::greedyProjectionMeshing(input, 100, 2.0, 2.5);

        // Write mesh to file
        pcl::io::saveOBJFile ( "greedyExample.obj", output);
        std::cout << "File saved as greedyExample.obj" << std::endl;

        // Load in old mesh for comparison
        pcl::io::loadOBJFile("greedyExampleChanged.obj", old_mesh );

    }

    ~Fix(){ std::cerr << "Cleaning up greedyProjectionMeshing objects" << std::endl; }

    volcart::testing::testingMesh mesh;
    pcl::PolygonMesh old_mesh ;
    pcl::PolygonMesh output ;
};


// Comparison function to check for accurate results, compare each point and each cell/face
BOOST_FIXTURE_TEST_CASE(compareMeshes, Fix)
{

    // Convert each mesh to obtain the correct points/data
    pcl::PointCloud<pcl::PointNormal> convOutputCloud;
    pcl::fromPCLPointCloud2(output.cloud, convOutputCloud);

    pcl::PointCloud<pcl::PointNormal> convOldCloud;
    pcl::fromPCLPointCloud2(old_mesh.cloud, convOldCloud);

    // Check size of data in both meshes
    BOOST_CHECK_EQUAL (convOutputCloud.points.size(), convOldCloud.points.size());

        // Otherwise, compare points

    // Check points in cloud
    for (int i = 0; i < convOutputCloud.points.size(); i++) {

        // 4th thing in data array is a type, so don't use it
        for (int m = 0; m < 3; m++ ) {

            BOOST_CHECK_EQUAL (convOutputCloud.points[i].data[m], convOldCloud.points[i].data[m]) ;
        }
    }


    // Compare size of polygon vectors
    BOOST_CHECK_EQUAL (output.polygons.size(), old_mesh.polygons.size());

    // Check faces
    // Iterate through each of the vertices of both new and old mesh and compare
    for (int i = 0; i < output.polygons.size(); i++) {

        for (int j = 0; j < output.polygons[i].vertices.size(); j++) {

            BOOST_CHECK_EQUAL (output.polygons[i].vertices[j] , old_mesh.polygons[i].vertices[j]);
        }
    }

    std::cout << std::endl << "The meshes are the same!" << std::endl;

    return;
}


