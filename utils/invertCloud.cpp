// invertcloud.cpp
// Seth Parker, Aug 2015

#include <iostream>
#include <fstream>

#include "volumepkg.h"
#include "vc_defines.h"


int main(int argc, char* argv[])
{
    if ( argc < 3 ) {
        std::cout << "Usage: vc_invertCloud volpkg [cloud].pcd" << std::endl;
        exit( -1 );
    }

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    std::string input_path = argv[ 2 ];

    if ( vpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << input_path << std::endl;

    // Load the cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input  (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_path, *input);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output  (new pcl::PointCloud<pcl::PointXYZRGB>);

    for ( auto pt = input->points.begin(); pt != input->points.end(); ++pt ) {
        pcl::PointXYZRGB point;
        point = *pt;
        if ( point.z != -1 ) point.z = vpkg.getNumberOfSlices() - 1 - point.z;
        output->push_back(point);
    }

    pcl::io::savePCDFileBinaryCompressed("inverted.pcd", *output);

    return 0;
} // end main

