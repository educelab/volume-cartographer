#ifndef ORDEREDPCDMESHER_H
#define ORDEREDPCDMESHER_H

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

namespace volcart {
    namespace meshing {

        int orderedPCDMesher(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string outFile);

    }// namespace meshing
}// namespace volcart

#endif