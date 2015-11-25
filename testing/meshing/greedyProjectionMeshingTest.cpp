//
// Created by Melissa Shankle on 10/26/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE greedyProjectionMeshing

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "vc_defines.h"
#include "shapes.h"
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
 *      - OBJ file to be loaded in as a PCL::PolygonMesh                                *
 *                                                                                      *
 *  Test-Specific Output:                                                               *
 *      Either a message stating the meshes are the same or error messages according    *
 *      to Boost.                                                                       *
 *                                                                                      *
 *  Miscellaneous:                                                                      *
 *      See the /testing/meshing wiki for more information on this test.                *
 *                                                                                      *
 ***************************************************************************************/

// General outline for test
//   Create new mesh using greedyProjectionMeshing
//   Take in input for test comparision
//   Convert both meshes to correct types
//   Compare new mesh with known mesh for equivalency
//   If errors occur, output them. Otherwise give success message.


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
        new_mesh = volcart::meshing::greedyProjectionMeshing(input, 100, 2.0, 2.5);

        //uncomment to enable inline obj file generation
        // Write mesh to file
        // pcl::io::saveOBJFile ( "greedyExample.obj", output);
        //std::cout << "File saved as greedyExample.obj" << std::endl;


        // Load in old mesh for comparison in the compareMeshes test case below
        pcl::io::loadOBJFile("greedyExample.obj", saved_mesh);

    }

    ~Fix(){ std::cerr << "Cleaning up greedyProjectionMeshing objects" << std::endl; }

    volcart::shapes::Plane mesh;
    pcl::PolygonMesh saved_mesh;
    pcl::PolygonMesh new_mesh;
};


// Comparison function to check for accurate results, compare each point and each cell/face
BOOST_FIXTURE_TEST_CASE(compareMeshes, Fix)
{

    // Convert each mesh to obtain the correct points/data
    pcl::PointCloud<pcl::PointNormal> convOutputCloud;
    pcl::fromPCLPointCloud2(new_mesh.cloud, convOutputCloud);

    pcl::PointCloud<pcl::PointNormal> convOldCloud;
    pcl::fromPCLPointCloud2(saved_mesh.cloud, convOldCloud);

    // Compare size of points in each of the PolygonMesh objects
    BOOST_CHECK_EQUAL (convOutputCloud.points.size(), convOldCloud.points.size());


    // Check points in cloud
    for (int i = 0; i < convOutputCloud.points.size(); i++) {

        // 4th element in data array is a type, so don't use it
        for (int m = 0; m < 3; m++ ) {

            BOOST_CHECK_EQUAL (convOutputCloud.points[i].data[m], convOldCloud.points[i].data[m]) ;
        }
    }


    // Compare size of polygons in each of the PolygonMesh objects
    BOOST_CHECK_EQUAL (new_mesh.polygons.size(), saved_mesh.polygons.size());

    // Checking faces for equality
    // Compare each of the vertices of both new and saved meshes
    for (int i = 0; i < new_mesh.polygons.size(); i++) {

        for (int j = 0; j < new_mesh.polygons[i].vertices.size(); j++) {

            BOOST_CHECK_EQUAL (new_mesh.polygons[i].vertices[j] , saved_mesh.polygons[i].vertices[j]);
        }
    }

    std::cout << std::endl << "The meshes are the same!" << std::endl;

    return;
}


