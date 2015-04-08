#ifndef ORDEREDPCDMESHER_H
#define ORDEREDPCDMESHER_H

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int orderedPCDMesher(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string outFile);

#endif