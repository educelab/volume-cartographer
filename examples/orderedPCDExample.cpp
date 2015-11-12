//
// Created by Ryan Taber on 11/12/15.
//


/* PURPOSE:
 *   Create a PolygonMesh from volcart::meshing::poissonReconstruction()
 *   Save resulting mesh as an obj file
 *   This file is loaded by poissonReconstructionTest
 */

#include "testing/testingMesh.h"
#include "orderedPCDMesher.h"

int orderedPCD(int argc, char** argv) {

    volcart::testing::testingMesh mesh;
    //Create point cloud from mesh
    pcl::PointCloud <pcl::PointNormal> pCloud = mesh.pointCloudXYZRGB();

    //convert pCloud to Ptr for orderedPCD() call
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    *cloud = pCloud;


    std::string outfile;

    if (argc == 1){  //no outfile filename provided
        outfile = "orderedPCDExample.pcd";
    }
    else if (argc == 2){
        outfile = argv[1];
    }
    else{
        return 1;
    }

    //call orderedPCDMesher()
    volcart::meshing::orderedPCDMesher(cloud, outfile);

    return 0;
}
