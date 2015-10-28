//
// Created by Media Team on 7/10/15.
//

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#ifndef VC_GREEDYPROJECTIONMESHING_H
#define VC_GREEDYPROJECTIONMESHING_H

namespace volcart {
    namespace meshing {

        pcl::PolygonMesh greedyProjectionMeshing ( pcl::PointCloud<pcl::PointNormal>::Ptr input, unsigned maxNeighbors = 100, double radius = 2.0, double radiusMultiplier = 2.5 );

    } // namespace meshing
} // namespace volcart

#endif //VC_GREEDYPROJECTIONMESHING_H
