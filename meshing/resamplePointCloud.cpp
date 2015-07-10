//
// Created by Media Team on 7/10/15.
//

#include "resamplePointCloud.h"

namespace volcart {
    namespace meshing {

        pcl::PointCloud<pcl::PointXYZRGBNormal> resamplePointCloud ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double radius ) {

            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr input_tree ( new pcl::search::KdTree<pcl::PointXYZRGB>() );

            pcl::PointCloud<pcl::PointXYZRGBNormal> mls_results;

            pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls_mesher;

            // Assign the input
            mls_mesher.setInputCloud(input);

            // Parameters
            mls_mesher.setComputeNormals(true);
            mls_mesher.setPolynomialFit(true);
            mls_mesher.setSearchMethod(input_tree);
            mls_mesher.setSearchRadius(radius);

            // Run MLS
            std::cerr << "volcart::meshing::running MLS..." << std::endl;
            mls_mesher.process(mls_results);
            std::cerr << "volcart::meshing::MLS complete..." << std::endl;

            // Filter NaN out of mls_points cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud (mls_results, mls_results, indices);
            indices.clear();
            pcl::removeNaNNormalsFromPointCloud (mls_results, mls_results, indices);
            indices.clear();

            return mls_results;
        }

    }
}