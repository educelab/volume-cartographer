//
// Created by Media Team on 7/10/15.
//

#include <iostream>

#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>

#ifndef VC_RESAMPLEPOINTCLOUD_H
#define VC_RESAMPLEPOINTCLOUD_H

namespace volcart {
    namespace meshing {
        pcl::PointCloud<pcl::PointNormal> resamplePointCloud ( pcl::PointCloud<pcl::PointNormal>::Ptr input, double radius = 2.0 );
    }
}

#endif //VC_RESAMPLEPOINTCLOUD_H
