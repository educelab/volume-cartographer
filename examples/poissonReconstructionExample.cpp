//
// Created by Ryan Taber on 11/12/15.
//

/* PURPOSE:
 *   Create a PolygonMesh from volcart::meshing::poissonReconstruction()
 *   Save resulting mesh as an obj file
 *   This file is loaded by poissonReconstructionTest
 */

#include "shapes.h"
#include "poissonReconstruction.h"
#include <pcl/io/obj_io.h>

int main(){

    //init shapes
    volcart::shapes::Plane Plane;
    volcart::shapes::Cube Cube;
    volcart::shapes::Arch Arch;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    //create point clouds from shapes
    pcl::PointCloud<pcl::PointNormal> in_PlanePointCloud = Plane.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_CubePointCloud = Cube.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_ArchPointCloud = Arch.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_SpherePointCloud = Sphere.pointCloudNormal();
    pcl::PointCloud<pcl::PointNormal> in_ConePointCloud = Cone.pointCloudNormal();

    //Convert point clouds to pointers for poissonReconstruction() call
    pcl::PointCloud<pcl::PointNormal>::Ptr in_PlaneCloudPtr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_CubeCloudPtr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_ArchCloudPtr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_SphereCloudPtr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr in_ConeCloudPtr(new pcl::PointCloud<pcl::PointNormal>);

    *in_PlaneCloudPtr = in_PlanePointCloud;
    *in_CubeCloudPtr = in_CubePointCloud;
    *in_ArchCloudPtr = in_ArchPointCloud;
    *in_SphereCloudPtr = in_SpherePointCloud;
    *in_ConeCloudPtr = in_ConePointCloud;

    //call poissonReconstruction on each of the point clouds() and assign to respective polygon mesh
    pcl::PolygonMesh out_PlanePolygonMesh = volcart::meshing::poissonReconstruction(in_PlaneCloudPtr);
    pcl::PolygonMesh out_CubePolygonMesh = volcart::meshing::poissonReconstruction(in_CubeCloudPtr);
    pcl::PolygonMesh out_ArchPolygonMesh = volcart::meshing::poissonReconstruction(in_ArchCloudPtr);
    pcl::PolygonMesh out_SpherePolygonMesh = volcart::meshing::poissonReconstruction(in_SphereCloudPtr);
    pcl::PolygonMesh out_ConePolygonMesh = volcart::meshing::poissonReconstruction(in_ConeCloudPtr);

    //write polygon mesh data to file
    pcl::io::saveOBJFile( "PlanePoissonReconstruction.obj", out_PlanePolygonMesh);
    pcl::io::saveOBJFile( "CubePoissonReconstruction.obj", out_CubePolygonMesh);
    pcl::io::saveOBJFile( "ArchPoissonReconstruction.obj", out_ArchPolygonMesh);
    pcl::io::saveOBJFile( "SpherePoissonReconstruction.obj", out_SpherePolygonMesh);
    pcl::io::saveOBJFile( "ConePoissonReconstruction.obj", out_ConePolygonMesh);

    std::cerr << "Files written: " << std::endl
              << "PlanePoissonReconstruction.obj" << std::endl
              << "CubePoissonReconstruction.obj" << std::endl
              << "ArchPoissonReconstruction.obj" << std::endl
              << "SpherePoissonReconstruction.obj" << std::endl
              << "ConePoissonReconstruction.obj" << std::endl;

    return EXIT_SUCCESS;

}