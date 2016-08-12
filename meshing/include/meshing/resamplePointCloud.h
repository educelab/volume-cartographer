//
// Created by Media Team on 7/10/15.
//
#pragma once

#include <iostream>

#include <pcl/common/common.h>

namespace volcart {
    namespace meshing {
        pcl::PointCloud<pcl::PointNormal> resamplePointCloud ( pcl::PointCloud<pcl::PointXYZ>::Ptr input, double radius = 2.0 );
    }
}
