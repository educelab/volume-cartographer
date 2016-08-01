//
// Created by Media Team on 7/10/15.
//

#include "meshing/greedyProjectionMeshing.h"

/*
 * See here for some documentation: http://www.pointclouds.org/documentation/tutorials/greedy_projection.php
 */

namespace volcart {
    namespace meshing {

        pcl::PolygonMesh greedyProjectionMeshing ( pcl::PointCloud<pcl::PointNormal>::Ptr input, unsigned maxNeighbors, double radius, double radiusMultiplier ) {

            // Make a Kd-tree for the input cloud
            pcl::search::KdTree<pcl::PointNormal>::Ptr input_tree (new pcl::search::KdTree<pcl::PointNormal>() );
            input_tree->setInputCloud(input);

            // Create the gpt instance and our output mesh
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedyProjection;
            pcl::PolygonMesh output;

            // Maximum distance between connected points
            greedyProjection.setSearchRadius(radius);

            // Max number of neighbors and the maximum distance from the center point
            greedyProjection.setMaximumNearestNeighbors(maxNeighbors);
            greedyProjection.setMu(radiusMultiplier);

            // Defaults for the rest
            greedyProjection.setMinimumAngle(M_PI/18); // 10 degrees - min angle in triangle
            greedyProjection.setMaximumAngle(2*M_PI); // 120 degrees - max angle in triangle
            greedyProjection.setMaximumSurfaceAngle(M_PI/4); // 45 degrees - don't connect points whose diff of normals are greater than this
            greedyProjection.setNormalConsistency(false); // if false, consider +normal and -normal for previous check

            // The data
            greedyProjection.setInputCloud(input);
            greedyProjection.setSearchMethod(input_tree);

            // Do the deed
            greedyProjection.reconstruct(output);

            return output;
        }

    } // namespace meshing
} // namespace volcart
