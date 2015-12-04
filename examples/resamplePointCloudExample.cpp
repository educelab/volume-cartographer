//
// Created by Ryan Taber on 11/16/15.
//

/*
 * Purpose: Run volcart::meshing::resamplePointCloud() and write results to file.
 *          Saved file will be read in by the resamplePointCloudTest.cpp file under
 *          v-c/testing/meshing.
 */

#include "vc_defines.h"
#include "shapes.h"
#include "resamplePointCloud.h"
#include <pcl/io/pcd_io.h>

int main() {

    //Construct a planar mesh and convert to <PointXYZ> point cloud
    volcart::shapes::Plane mesh;
    pcl::PointCloud <pcl::PointXYZ> pCloud = mesh.pointCloudXYZ();

    //convert pCloud to Ptr for resample() call
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pCloud;

    //init PointCloud that will store the resample() call results
    pcl::PointCloud <pcl::PointNormal> resampledCloud;

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

    //set search radius based on 2.5(avgDistance) and call resample()
    double radius = 2.5 * avgDistance;

    //call resample PC twice
    resampledCloud = volcart::meshing::resamplePointCloud(cloud, radius);

    //write the resampled PointCloud data to file
    pcl::io::savePCDFile("resampleExample.pcd", resampledCloud);


    std::cerr << "File written to resampleExample.pcd" << std::endl;

    return 0;
}

