//
// Created by Ryan Taber on 11/12/15.
//

/* PURPOSE:
 *   Create a PolygonMesh from volcart::meshing::poissonReconstruction()
 *   Save resulting mesh as an obj file
 *   This file is loaded by poissonReconstructionTest
 */

#include "testingMesh.h"
#include "meshing/poissonReconstruction.h"
#include <pcl/io/obj_io.h>

int poisson(int argc, char** argv){



    volcart::testing::testingMesh mesh;
    pcl::PointCloud<pcl::PointNormal> pCloud = mesh.pointCloudNormal();

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    *cloud = pCloud;

    pcl::PolygonMesh polyMesh;
    polyMesh = volcart::meshing::poissonReconstruction(cloud);

    if (argc == 1){  //no outfile filename provided
        pcl::io::saveOBJFile( "poissonSurface.obj", polyMesh);
    }
    else if (argc == 2){
        pcl::io::saveOBJFile(argv[1], polyMesh);
    }
    else{
        return 1;
    }

    return 0;

}