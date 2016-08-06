//
// Created by Ryan Taber on 11/12/15.
//


/* PURPOSE:
 *   Create a point cloud with points of the XYZRGB variety
 *   Save the resulting pcd using the volcart::meshing::orderedPCDMesher()
 *   The output created here is loaded by orderedPCDMesherTest for later comparison
 */

#include "meshing/orderedPCDMesher.h"
#include "common/shapes/Plane.h"
#include "common/shapes/Arch.h"
#include "common/shapes/Cube.h"
#include "common/shapes/Sphere.h"
#include "common/shapes/Cone.h"

int main() {

    // Init Shape Meshes
    volcart::shapes::Plane Plane;
    volcart::shapes::Arch Arch;
    volcart::shapes::Cube Cube;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    //Create point cloud from mesh
    pcl::PointCloud <pcl::PointXYZRGB> PlanePointCloud = Plane.pointCloudXYZRGB();
    pcl::PointCloud <pcl::PointXYZRGB> CubePointCloud = Cube.pointCloudXYZRGB();
    pcl::PointCloud <pcl::PointXYZRGB> ArchPointCloud = Arch.pointCloudXYZRGB();
    pcl::PointCloud <pcl::PointXYZRGB> SpherePointCloud = Sphere.pointCloudXYZRGB();
    pcl::PointCloud <pcl::PointXYZRGB> ConePointCloud = Cone.pointCloudXYZRGB();


    //convert ShapePointClouds to Ptrs for orderedPCDMesher() calls
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointerToPlaneCloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointerToCubeCloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointerToArchCloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointerToSphereCloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointerToConeCloud(new pcl::PointCloud <pcl::PointXYZRGB>);

    //Assign Shape Cloud Pointers
    *PointerToPlaneCloud = PlanePointCloud;
    *PointerToCubeCloud = CubePointCloud;
    *PointerToArchCloud = ArchPointCloud;
    *PointerToSphereCloud = SpherePointCloud;
    *PointerToConeCloud = ConePointCloud;

    //make calls to orderedPCDMesher()
    volcart::meshing::orderedPCDMesher(PointerToPlaneCloud, "PlaneOrderedPCDMesher.ply");
    volcart::meshing::orderedPCDMesher(PointerToCubeCloud, "CubeOrderedPCDMesher.ply");
    volcart::meshing::orderedPCDMesher(PointerToArchCloud, "ArchOrderedPCDMesher.ply");
    volcart::meshing::orderedPCDMesher(PointerToSphereCloud, "SphereOrderedPCDMesher.ply");
    volcart::meshing::orderedPCDMesher(PointerToConeCloud, "ConeOrderedPCDMesher.ply");

    std::cerr << "Files written:" << std::endl
              << "PlaneOrderedPCDMesher.ply" << std::endl
              << "CubeOrderedPCDMesher.ply" << std::endl
              << "ArchOrderedPCDMesher.ply" << std::endl
              << "SphereOrderedPCDMesher.ply" << std::endl
              << "ConeOrderedPCDMesher.ply" << std::endl;

    return 0;
}
