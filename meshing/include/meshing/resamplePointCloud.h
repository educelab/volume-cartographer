//
// Created by Media Team on 7/10/15.
//

#include <iostream>

#include <pcl/common/common.h>

#ifndef VC_RESAMPLEPOINTCLOUD_H
#define VC_RESAMPLEPOINTCLOUD_H

namespace volcart {
    namespace meshing {
        pcl::PointCloud<pcl::PointNormal> resamplePointCloud ( pcl::PointCloud<pcl::PointXYZ>::Ptr input, double radius = 2.0 );
    }
}

#endif //VC_RESAMPLEPOINTCLOUD_H