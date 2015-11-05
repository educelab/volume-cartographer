//
// Created by Melissa Shankle on 10/27/15.
//

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include "pcl/conversions.h"

#include "vc_defines.h"
#include "testing/testingMesh.h"
#include "greedyProjectionMeshing.h"

using namespace volcart::meshing;
void compareMeshes (const ::pcl::PolygonMesh &output, const ::pcl::PolygonMesh &old_mesh);

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
    pcl::PointCloud<pcl::PointNormal> cloud_PointNormal = mesh.pointCloudNormal();

    // Create pointer for the input point cloud to pass to greedyProjectionMeshing
    pcl::PointCloud<pcl::PointNormal>::Ptr input( new pcl::PointCloud<pcl::PointNormal>);
    *input = cloud_PointNormal;


    // Call function from namespace in header file
    std::cout << "Being greedy..." << std::endl;
    pcl::PolygonMesh output = greedyProjectionMeshing(input, 100, 2.0, 2.5);

    // Write mesh to file
    pcl::io::saveOBJFile ( "greedyExample.obj", output);

    std::cout << "File saved as greedyExample.obj" << std::endl;
    //std::cout << input->size() << std::endl;


    pcl::PolygonMesh old_mesh ;
    pcl::io::loadOBJFile(argv[1], old_mesh );
    compareMeshes(output, old_mesh);

   return 0;
}

// Comparison function to check for accurate results, compare each point and each cell
void compareMeshes (const ::pcl::PolygonMesh &output, const ::pcl::PolygonMesh &old_mesh) {

    //std::cout << output.cloud << endl;
    //std::cout << old_mesh.cloud << endl;

    //try this conversion stuff out
    pcl::PointCloud<pcl::PointNormal> convOutputCloud;
    pcl::fromPCLPointCloud2(output.cloud, convOutputCloud);

    pcl::PointCloud<pcl::PointNormal> convOldCloud;
    pcl::fromPCLPointCloud2(output.cloud, convOldCloud);

    //check size of data in both meshes

    /* current sizes: output = 1200 old_mesh=600*/
    if (convOutputCloud.points.size() != convOldCloud.points.size()){
        std::cout << "cloud.data.size() mismatch. " << std::endl;
        //return;
    }
    //Otherwise, compare points
    else {
        // Check points in cloud
        for (int i = 0; i < convOutputCloud.points.size(); i++) {

            // 4th thing in data array is a type, so don't use it
            for (int m = 0; m < 3; m++ ) {

                if (convOutputCloud.points[i].data[m] != convOldCloud.points[i].data[m]) {
                    std::cout << "Cloud points do not match at points[" << i << "].data[" << m << "]" << std::endl;
                    std::cout << convOutputCloud.points[i].data[0] << convOutputCloud.points[i].data[1] <<
                    convOutputCloud.points[i].data[2] << convOutputCloud.points[i].data[3]
                    << " || " << convOldCloud.points[i].data[0] << convOldCloud.points[i].data[1] <<
                    convOldCloud.points[i].data[2] << convOldCloud.points[i].data[3] << std::endl;

                }
            }
        }
    }


    //std::cout << "output size " << output.polygons.size() << " oldmesh size " << old_mesh.polygons.size() << endl;
    if (output.polygons.size() != old_mesh.polygons.size()) {
        std::cout << "Vectors are different sizes, therefore they're not equal." << endl;
        return;
    }

    // Check faces
    // Iterate through each of the vertices of both new and old mesh and compare
    // Example: output.polygons[0].vertices[0]; // Very first vertices
    for (int i = 0; i < output.polygons.size(); i++) {

        for (int j = 0; j < output.polygons[i].vertices.size(); j++) {

            //std::cout << "Polygon " << i << " at vertex " << j << endl;
            //std::cout << output.polygons[i].vertices[j] << " " << old_mesh.polygons[i].vertices[j] << endl;

            if (output.polygons[i].vertices[j] != old_mesh.polygons[i].vertices[j]) {
                std::cout << "Difference at polygon " << i << " at vertex " << j << endl;

                return;
            }
        }
    }

    std::cout << "The meshes are the same!" << endl;
}