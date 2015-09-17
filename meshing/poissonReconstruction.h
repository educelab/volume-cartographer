//
// Created by Media Team on 7/10/15.
//

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/poisson.h>

#ifndef VC_POISSONRECONSTRUCITON_H
#define VC_POISSONRECONSTRUCTION_H

namespace volcart {
    namespace meshing {

    pcl::PolygonMesh poissonReconstruction ( pcl::PointCloud<pcl::PointNormal>::Ptr input );

    }
}

#endif // VC_POISSONRECONSTRUCTION_H
