//
// Created by Media Team on 7/10/15.
//

#include "poissonReconstruction.h"

namespace volcart {
    namespace meshing {

        pcl::PolygonMesh poissonReconstruction ( pcl::PointCloud<pcl::PointNormal>::Ptr input ){
    
            std::cout << "Begin poisson reconstruction" << std::endl;
            pcl::Poisson<pcl::PointNormal> poisson;
            /*
             * This integer is the maximum depth of the tree that will be used for surface reconstruction.
             * Running at depth d corresponds to solving on a voxel grid whose resolution is no larger
             * than 2^d x 2^d x 2^d. Note that since the reconstructor adapts the octree to the
             * sampling density, the specified reconstruction depth is only an upper bound. Default is 8, but 9/10 rec.
             */
            poisson.setDepth(9);
            poisson.setInputCloud(input);
            pcl::PolygonMesh mesh;
            poisson.reconstruct(mesh);

            return mesh;
        }

    } // namespace meshing
} // namespace volcart
