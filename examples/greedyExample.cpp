//
// Created by Melissa Shankle on 10/27/15.
//

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>

#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "greedyProjectionMeshing.h"

using namespace volcart::meshing;

int main(int argc, char* argv[]) {

    /* If arguments changed from the set values in greedyProjectionMeshing.h
    if ( argc != 5 ) {
        std::cout << "Incorrect arguments" << endl;
        return 1;
    }
    unsigned max = argv[2];
    double radius = argv[3];
    double radiusMultiplier = argv[4];
    */

    // Take input (a Point Cloud)
    volcart::testing::testingMesh mesh;
    pcl::PointCloud<pcl::PointNormal> cloud_PointNormal = mesh.pointCloud();

    // Create pointer for the input point cloud to pass to greedyProjectionMeshing
    pcl::PointCloud<pcl::PointNormal>::Ptr input(&cloud_PointNormal);


    // Call function from namespace in header file
    std::cout << "Being greedy..." << std::endl;
    pcl::PolygonMesh output = greedyProjectionMeshing(input, 100, 2.0, 2.5);

    // Write mesh to file
    pcl::io::saveOBJFile ( "greedyExample.obj", output);

    std::cout << "File saved as greedyExample.obj" << std::endl;
    //std::cout << input->size() << std::endl;



    return 0;
}
