#include <list>
#include <tuple>
#include <numeric>
#include <memory>
#include <random>

#include "localResliceParticleSim.h"
#include "common.h"


using namespace volcart::segmentation;

LocalResliceSegmentation::LocalResliceSegmentation(VolumePkg& pkg) :
        pkg_(pkg), startIndex_(0), endIndex_(0) { }

pcl::PointCloud<pcl::PointXYZRGB>
LocalResliceSegmentation::segmentLayer(const bool showVisualization,
                                       const int32_t startIndex,
                                       const int32_t endIndex,
                                       const int32_t stepNumLayers,
                                       const double  derivativeTolerance,
                                       const int32_t keepNumMaxima,
                                       const int32_t numRandomTries)
{
    // Account for zero-indexing and slices lost in calculating normal vector
    startIndex_ = startIndex;
    endIndex_ = pkg_.getNumberOfSlices() - 3;
    if (endIndex != kDefaultEndIndex) {
        endIndex_ = endIndex;
    }

    // Seed random number generator
    std::random_device rd;
    std::default_random_engine mt(rd());

    // ChainMesh that holds the segment
    VoxelVec currentPos;
    auto path = pkg_.openCloud(); 
    for (const auto& p : *path) {
        currentPos.emplace_back(p.x, p.y, p.z);
    }
    int32_t N = currentPos.size();
    ChainMesh mesh(N, endIndex_ - startIndex_);
    mesh.addPositions(currentPos);

    for (int32_t sliceIndex = startIndex_; sliceIndex <= endIndex_;
         sliceIndex += stepNumLayers) {

        // First chain object
        Chain chain(pkg_, currentPos, sliceIndex);

        // Get current derivatives
        int32_t N = chain.size();
        std::vector<double> initDerivs = chain.curve().deriv();

		std::cout << "slice: " << sliceIndex << std::endl;

        if (showVisualization) {
            chain.draw();
            cv::waitKey(0);
        }

        std::vector<VoxelVec> choiceSpace =
            chain.stepAll(stepNumLayers, keepNumMaxima);

        // Get the first set of positions (all the 'best' positions independent
        // of neighborhood constraints)
        VoxelVec bestPositions;
        bestPositions.reserve(N);
        for (const auto& v : choiceSpace) {
            bestPositions.push_back(v[0]);
        }

        // And evaluate it
        std::vector<double> combinationDerivs = chain.curve().deriv();
        double minDerivativeDiff =
            l2_difference_norm(combinationDerivs, initDerivs);

        for (int32_t i = 0; i < numRandomTries; ++i) {
            combinationDerivs.clear();

            // 1. Generate random positions vector from the indices in this
            VoxelVec positions;
            positions.reserve(N);
            for (const auto& v : choiceSpace) {
                std::uniform_int_distribution<int32_t> dist(0, v.size() - 1);
                positions.push_back(v[dist(mt)]);
            }
            //std::cout << "positions: " << positions << std::endl;

            // 2. Generate derivatives from positions
			combinationDerivs = chain.curve().deriv();
            //std::cout << "combination derivs: " << combinationDerivs << std::endl;
    
            // 2. Calculate l2 norm of this and previous positions and set
            //    min accordingly
            double norm = l2_difference_norm(combinationDerivs, initDerivs);
            //std::cout << "norm: " << norm << std::endl;
            if (norm - minDerivativeDiff < derivativeTolerance) {
                minDerivativeDiff = norm;
                bestPositions = positions;
            }
            //std::cout << std::endl;
        }
        std::cout << "minDerivDiff: " << minDerivativeDiff << std::endl;
        mesh.addPositions(bestPositions);
        currentPos = bestPositions;
    }

    return mesh.exportAsPointCloud();
}
