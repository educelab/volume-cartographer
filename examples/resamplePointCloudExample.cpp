//
// Created by Ryan Taber on 11/16/15.
//

/*
 * Purpose: Run volcart::meshing::resamplePointCloud() and write results to file.
 *          Saved files will be read in by the resamplePointCloudTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "vc_defines.h"
#include "shapes.h"
#include "resamplePointCloud.h"
#include <pcl/io/pcd_io.h>

//helper function
double CalculateRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

int main() {

    //Construct a shapes and convert to <PointXYZ> point cloud
    volcart::shapes::Plane Plane;
    volcart::shapes::Cube Cube;
    volcart::shapes::Arch Arch;
    volcart::shapes::Sphere Sphere;
    volcart::shapes::Cone Cone;

    pcl::PointCloud <pcl::PointXYZ> PlaneCloudXYZ = Plane.pointCloudXYZ();
    pcl::PointCloud <pcl::PointXYZ> CubeCloudXYZ = Cube.pointCloudXYZ();
    pcl::PointCloud <pcl::PointXYZ> ArchCloudXYZ = Arch.pointCloudXYZ();
    pcl::PointCloud <pcl::PointXYZ> SphereCloudXYZ = Sphere.pointCloudXYZ();
    pcl::PointCloud <pcl::PointXYZ> ConeCloudXYZ = Cone.pointCloudXYZ();

    //convert shape Point Clouds to Ptr for resample() calls
    pcl::PointCloud<pcl::PointXYZ>::Ptr PlaneCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr CubeCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ArchCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr SphereCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ConeCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

    //transfer data to cloud ptrs
    *PlaneCloudPtr = PlaneCloudXYZ;
    *CubeCloudPtr = CubeCloudXYZ;
    *ArchCloudPtr = ArchCloudXYZ;
    *SphereCloudPtr = SphereCloudXYZ;
    *ConeCloudPtr = ConeCloudXYZ;

    //init PointClouds that will store the resample() call results
    pcl::PointCloud <pcl::PointNormal> out_PlaneResampledPointCloud;
    pcl::PointCloud <pcl::PointNormal> out_CubeResampledPointCloud;
    pcl::PointCloud <pcl::PointNormal> out_ArchResampledPointCloud;
    pcl::PointCloud <pcl::PointNormal> out_SphereResampledPointCloud;
    pcl::PointCloud <pcl::PointNormal> out_ConeResampledPointCloud;

    //make calls to resample()
    out_PlaneResampledPointCloud = volcart::meshing::resamplePointCloud(PlaneCloudPtr, CalculateRadius(PlaneCloudPtr));
    out_CubeResampledPointCloud = volcart::meshing::resamplePointCloud(CubeCloudPtr, CalculateRadius(CubeCloudPtr));
    out_ArchResampledPointCloud = volcart::meshing::resamplePointCloud(ArchCloudPtr, CalculateRadius(ArchCloudPtr));
    out_SphereResampledPointCloud = volcart::meshing::resamplePointCloud(SphereCloudPtr, CalculateRadius(SphereCloudPtr));
    out_ConeResampledPointCloud = volcart::meshing::resamplePointCloud(ConeCloudPtr, CalculateRadius(ConeCloudPtr));

    //write the resampled PointCloud data to file
    pcl::io::savePCDFile("PlaneResamplePointCloudExample.pcd", out_PlaneResampledPointCloud);
    pcl::io::savePCDFile("CubeResamplePointCloudExample.pcd", out_CubeResampledPointCloud);
    pcl::io::savePCDFile("ArchResamplePointCloudExample.pcd", out_ArchResampledPointCloud);
    pcl::io::savePCDFile("SphereResamplePointCloudExample.pcd", out_SphereResampledPointCloud);
    pcl::io::savePCDFile("ConeResamplePointCloudExample.pcd", out_ConeResampledPointCloud);

    std::cerr << "Files written:" << std::endl
              << "PlaneResamplePointCloudExample.pcd" << std::endl
              << "CubeResamplePointCloudExample.pcd" << std::endl
              << "ArchResamplePointCloudExample.pcd" << std::endl
              << "SphereResamplePointCloudExample.pcd" << std::endl
              << "ConeResamplePointCloudExample.pcd" << std::endl;

    return EXIT_SUCCESS;
}


double CalculateRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

  /*
   * Determine the ideal search radius for the resample call
   */

    float avgDistance = 0;
    int count = 0;

    //This is assuming a quadratic basis for the points
    //For linear, multiply the avgDistance by 1.2
    //Get the avg distance from points in the point cloud
    for (auto a = cloud->begin(); a != cloud->end(); a++){
        for (auto b = cloud->begin(); b != cloud->end(); b++){

            float pDistance = 0;
            if (a != b)
                pDistance = sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));

            count++;
            avgDistance += pDistance;
        }
    }

    avgDistance /= count;

    //set search radius based on 2.5(avgDistance)
    return 2.5 * avgDistance;
}