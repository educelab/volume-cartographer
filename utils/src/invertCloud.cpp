// invertcloud.cpp
// Seth Parker, Aug 2015

#include <iostream>
#include <fstream>

#include "volumepkg/volumepkg.h"
#include "common/vc_defines.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <boost/filesystem/path.hpp>

namespace fs = boost::filesystem;

int main(int argc, char* argv[])
{
    if ( argc < 3 ) {
        std::cout << "Usage: vc_invertCloud volpkg [input].pcd [output].pcd" << std::endl;
        exit( -1 );
    }

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    fs::path input_path = argv[ 2 ];
    fs::path output_path = argv[ 3 ];

    if ( vpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << input_path << std::endl;

    // Load the cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input  (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_path.string(), *input);

    for ( auto pt = input->points.begin(); pt != input->points.end(); ++pt ) {
        if ( pt->z != -1 ) pt->z = vpkg.getNumberOfSlices() - 1 - pt->z;
    }

    pcl::io::savePCDFileBinaryCompressed(output_path.string(), *input);

    return 0;
} // end main

