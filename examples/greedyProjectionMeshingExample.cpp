//
// Created by Melissa Shankle on 10/27/15.
//

/*
 * Purpose: Run volcart::meshing::greedyProjectionMeshing() and write results to file.
 *          Saved file will be read in by the greedyProjectionMeshingTest.cpp file under
 *          v-c/testing/meshing.
 */


#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include "shapes.h"
#include "greedyProjectionMeshing.h"

using namespace volcart::meshing;

int main(int argc, char* argv[]) {

    //init shapes
    volcart::shapes::Plane Plane;
    volcart::shapes::Cube Cube;
    volcart::shapes::Arch Arch;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    //convert shaped meshes to point cloud
    pcl::PointCloud<pcl::PointNormal> in_PlanePointNormalCloud = Plane.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_CubePointNormalCloud = Cube.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_ArchPointNormalCloud = Arch.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_SpherePointNormalCloud = Sphere.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_ConePointNormalCloud = Cone.pointCloudNormal();

    // Create ptr to point clouds and fill --> needed for the call to greedyProjectionMeshing()
    pcl::PointCloud<pcl::PointNormal>::Ptr in_PlanePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_CubePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_ArchPointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_SpherePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_ConePointNormalCloudPtr( new pcl::PointCloud<pcl::PointNormal>);

    *in_PlanePointNormalCloudPtr = in_PlanePointNormalCloud;
    *in_CubePointNormalCloudPtr = in_CubePointNormalCloud;
    *in_ArchPointNormalCloudPtr = in_ArchPointNormalCloud;
    *in_SpherePointNormalCloudPtr = in_SpherePointNormalCloud;
    *in_ConePointNormalCloudPtr = in_ConePointNormalCloud;

    // Call function from
    pcl::PolygonMesh out_PlanePolyMesh  = greedyProjectionMeshing(in_PlanePointNormalCloudPtr,  100, 2.0, 2.5);
    pcl::PolygonMesh out_CubePolyMesh   = greedyProjectionMeshing(in_CubePointNormalCloudPtr,   100, 2.0, 2.5);
    pcl::PolygonMesh out_ArchPolyMesh   = greedyProjectionMeshing(in_ArchPointNormalCloudPtr,   100, 2.0, 2.5);
    pcl::PolygonMesh out_SpherePolyMesh = greedyProjectionMeshing(in_SpherePointNormalCloudPtr, 100, 2.0, 2.5);
    pcl::PolygonMesh out_ConePolyMesh   = greedyProjectionMeshing(in_ConePointNormalCloudPtr,   100, 2.0, 2.5);


    // Write meshes to file
    pcl::io::saveOBJFile ( "PlaneGreedyProjectionMeshing.obj", out_PlanePolyMesh);
    pcl::io::saveOBJFile ( "CubeGreedyProjectionMeshing.obj", out_CubePolyMesh);
    pcl::io::saveOBJFile ( "ArchGreedyProjectionMeshing.obj", out_ArchPolyMesh);
    pcl::io::saveOBJFile ( "SphereGreedyProjectionMeshing.obj", out_SpherePolyMesh);
    pcl::io::saveOBJFile ( "ConeGreedyProjectionMeshing.obj", out_ConePolyMesh);

    return EXIT_SUCCESS;
}

