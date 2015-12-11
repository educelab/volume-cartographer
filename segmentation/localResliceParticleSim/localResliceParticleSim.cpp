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
    std::mt19937 mt(rd());

    // ChainMesh that holds the segment
    VoxelVec currentPos;
    auto path = pkg_.openCloud(); 
    for (const auto& p : *path) {
        currentPos.emplace_back(p.x, p.y, p.z);
    }
    int32_t N = currentPos.size();
    ChainMesh mesh(N, endIndex_ - startIndex_);
    mesh.addPositions(currentPos);

    // Need to start at startIndex_ + stepNumLayers since we already added the intial
    // positions to the chainmesh.
    for (int32_t sliceIndex = startIndex_ + stepNumLayers; sliceIndex <= endIndex_;
         sliceIndex += stepNumLayers) {

        // First chain object
        Chain currentChain(pkg_, currentPos, sliceIndex);

        // Get current derivatives
        int32_t N = currentChain.size();
        std::vector<double> currentDerivs;
        currentDerivs.reserve(N);
        for (int32_t i = 0; i < N; ++i) {
            currentDerivs.push_back(fivePointStencil(i, currentChain.positions()));
        }

		std::cout << "slice: " << sliceIndex << std::endl;

        if (showVisualization) {
            currentChain.draw();
            cv::waitKey(0);
        }

        std::vector<VoxelVec> choiceSpace =
            currentChain.stepAll(stepNumLayers, keepNumMaxima);

        // Get the first set of positions (all the 'best' positions independent
        // of neighborhood constraints)
        VoxelVec bestPositions;
        bestPositions.reserve(N);
        for (const auto& v : choiceSpace) {
            bestPositions.push_back(v[0]);
        }

        // And evaluate it
        std::vector<double> combinationDerivs;
        combinationDerivs.reserve(N);
        for (int32_t i = 0; i < N; ++i) {
            combinationDerivs.push_back(fivePointStencil(i, bestPositions));
        }
        //std::cout << "best derivs: " << combinationDerivs << std::endl;
        double minDerivativeDiff =
            l2_difference_norm(combinationDerivs, currentDerivs);
        //std::cout << "initial deriv L2: " << minDerivativeDiff << std::endl;

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
            for (int32_t i = 0; i < N; ++i) {
                combinationDerivs.push_back(fivePointStencil(i, positions));
            }
            //std::cout << "combination derivs: " << combinationDerivs << std::endl;
    
            // 2. Calculate l2 norm of this and previous positions and set
            //    min accordingly
            double norm = l2_difference_norm(combinationDerivs, currentDerivs);
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

// Uses the five point stencil equation as given here:
// https://en.wikipedia.org/wiki/Five-point_stencil
double LocalResliceSegmentation::fivePointStencil(
		const uint32_t center, const VoxelVec& ps) const
{
    // Average difference in x dimension across entire chain
    double avgXDiff = 0;
    for (uint32_t i = 1; i < ps.size(); ++i) {
        avgXDiff += (ps[i](VC_INDEX_X) - ps[i-1](VC_INDEX_X));
    }
    avgXDiff /= ps.size();

    // Fit curve to new predicted ps.
    FittedCurve<>::ScalarVector xs, ys;
    xs.reserve(ps.size());
    ys.reserve(ps.size());
    for (const auto& p : ps) {
        xs.push_back(p(VC_INDEX_X));
        ys.push_back(p(VC_INDEX_Y));
    }
    FittedCurve<> f(xs, ys);

    // Take care of any out of bounds accesses
    /*
    double firstPairXDiff = ps[1](VC_INDEX_X) - ps[0](VC_INDEX_X);
    double lastPairXDiff = ps[ps.size() - 1](VC_INDEX_X) - ps[ps.size() - 2](VC_INDEX_X);
    double twoBefore, oneBefore, oneAfter, twoAfter;
    if (center == 0) {
        twoBefore = f.at(ps[0](VC_INDEX_X) - 2 * firstPairXDiff);
        oneBefore = f.at(ps[0](VC_INDEX_X) - firstPairXDiff);
        oneAfter  = ps[center + 1](VC_INDEX_Y);
        twoAfter  = ps[center + 2](VC_INDEX_Y);
    } else if (center == 1) {
        twoBefore = f.at(ps[0](VC_INDEX_X) - firstPairXDiff);
        oneBefore = ps[0](VC_INDEX_Y);
        oneAfter  = ps[center + 1](VC_INDEX_Y);
        twoAfter  = ps[center + 2](VC_INDEX_Y);
    } else if (center == ps.size() - 2) {
        twoBefore = ps[center - 2](VC_INDEX_Y);
        oneBefore = ps[center - 1](VC_INDEX_Y);
        oneAfter  = ps[ps.size() - 1](VC_INDEX_Y);
        twoAfter  = f.at(ps[ps.size() - 1](VC_INDEX_X) + lastPairXDiff);
    } else if (center == ps.size() - 1) {
        twoBefore = ps[center - 2](VC_INDEX_Y);
        oneBefore = ps[center - 1](VC_INDEX_Y);
        oneAfter  = f.at(ps[ps.size() - 1](VC_INDEX_X) + lastPairXDiff);
        twoAfter  = f.at(ps[ps.size() - 1](VC_INDEX_X) + 2 * lastPairXDiff);
    } else {
        twoBefore = ps[center - 2](VC_INDEX_Y);
        oneBefore = ps[center - 1](VC_INDEX_Y);
        oneAfter  = ps[center + 1](VC_INDEX_Y);
        twoAfter  = ps[center + 2](VC_INDEX_Y);
    }

    return (-twoAfter + 8 * oneAfter - 8 * oneBefore + twoBefore) / (12 * avgXDiff);
    */

    auto before2 = f.at(ps[center](VC_INDEX_X) - 2 * avgXDiff).second;
    auto before1 = f.at(ps[center](VC_INDEX_X) - 1 * avgXDiff).second;
    auto after1  = f.at(ps[center](VC_INDEX_X) + 1 * avgXDiff).second;
    auto after2  = f.at(ps[center](VC_INDEX_X) + 2 * avgXDiff).second;
    return (-after2 + 8 * after1 - 8 * before1 + before2) / (12 * avgXDiff);
}
