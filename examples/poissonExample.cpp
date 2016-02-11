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

int main(/*int argc, char** argv*/){

    //Create planar mesh and point cloud from mesh
    volcart::shapes::Plane mesh;
    pcl::PointCloud<pcl::PointNormal> pCloud = mesh.pointCloudNormal();

    //Convert point cloud to ptr for poissonReconstruction() call
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    *cloud = pCloud;

    //call poissonRecon(), assign result and write to file
    pcl::PolygonMesh polyMesh;
    polyMesh = volcart::meshing::poissonReconstruction(cloud);
    pcl::io::saveOBJFile( "poissonExample.obj", polyMesh);

//
//    if (argc == 1){  //no outfile filename provided
//        pcl::io::saveOBJFile( "poissonSurface.obj", polyMesh);
//    }
//    else if (argc == 2){
//        pcl::io::saveOBJFile(argv[1], polyMesh);
//    }
//    else{
//        return 1;
//    }

    std::cerr << "File written to poissonExample.obj" << std::endl;

    return 0;

}