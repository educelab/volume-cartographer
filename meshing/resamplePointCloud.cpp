//
// Created by Media Team on 7/10/15.
//

#include "resamplePointCloud.h"

namespace volcart {
    namespace meshing {

        pcl::PointCloud<pcl::PointNormal> resamplePointCloud ( pcl::PointCloud<pcl::PointXYZ>::Ptr input, double radius ) {

            pcl::search::KdTree<pcl::PointXYZ>::Ptr input_tree ( new pcl::search::KdTree<pcl::PointXYZ>() );

            pcl::PointCloud<pcl::PointNormal> mls_results;

            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls_mesher;

            // Assign the input
            mls_mesher.setInputCloud(input);

            // Parameters
            mls_mesher.setComputeNormals(true);
            mls_mesher.setPolynomialFit(true);
            mls_mesher.setPolynomialOrder(4);   //See paper referenced by PCL in wiki. Recommends 3 or 4.
            mls_mesher.setSearchMethod(input_tree);
            mls_mesher.setSearchRadius(radius);

            // Upsampling: Testing Various Methods

//            //NONE (base case - no upsampling performed)
//              mls_mesher.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::NONE);
//
//            //SAMPLE LOCAL PLANE
//            mls_mesher.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
//            mls_mesher.setUpsamplingRadius(5);
//            mls_mesher.setUpsamplingStepSize(2.5);

//            //RANDOM UNIFORM DENSITY
//            mls_mesher.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
//            mls_mesher.setPointDensity(5);

            //VOXEL GRID DILATION
            mls_mesher.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
            //mls_mesher.setDilationVoxelSize(1.0);
            //mls_mesher.setDilationIterations(1);

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
