//
// Created by Media Team on 7/10/15.
//

#include "poissonReconstruction.h"

namespace volcart {
    namespace meshing {

        pcl::PolygonMesh poissonReconstruction ( pcl::PointCloud<pcl::PointNormal>::Ptr input ){
    
            std::cout << "Begin poisson reconstruction" << std::endl;
            pcl::Poisson<pcl::PointNormal> poisson;
            poisson.setDepth(9);
            poisson.setInputCloud(input);
            pcl::PolygonMesh mesh;
            poisson.reconstruct(mesh);

            return mesh;
        }

    } // namespace meshing
} // namespace volcart
