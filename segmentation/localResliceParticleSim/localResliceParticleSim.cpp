#include <list>
#include <tuple>

#include "localResliceParticleSim.h"
#include "common.h"

using namespace volcart::segmentation;

LocalResliceSegmentation::LocalResliceSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr segpath, VolumePkg& pkg) :
        startIndex_(0), endIndex_(0), pkg_(pkg)
{
    // Construct chain of particles
    currentChain_ = Chain(segpath);
    mesh_ = ChainMesh();
}


pcl::PointCloud<pcl::PointXYZRGB>
LocalResliceSegmentation::segmentLayer(const double driftTolerance,
                                       const int32_t neighborhoodRadius,
                                       const int32_t stepsBeforeReslice,
                                       const int32_t startOffset,
                                       const int32_t endOffset,
                                       const int32_t stepNumLayers,
                                       const int32_t maxIterations)
{
    startIndex_ = currentChain_.zIndex() + startOffset;
    // Account for zero-indexing and slices lost in calculating normal vector
    endIndex_ = pkg_.getNumberOfSlices() - 3 - endOffset;

    // Set mesh size
    mesh_.setSize(currentChain_.size(), endIndex_ - startIndex_);

    // Go through every iteration (from start to end index)
    for (int32_t iter = startIndex_; iter < endIndex_; ++iter) {

        // Get predicted directions and positions
        std::vector<Direction> predictedDirections;
        std::vector<cv::Vec3f> predictedPositions;
        std::tie(predictedDirections, predictedPositions) = currentChain_.stepAll();
        auto N = int32_t(predictedPositions.size());

        // Get XY drift of newPositions from currentPositions
        auto xyDrift = std::vector<double>(predictedPositions.size());
        for (auto i = 0; i < N; ++i) {
            xyDrift[i] = cv::norm(predictedPositions[i] - currentChain_.at(i).position());
        }

        // "Bad" step indices. A "bad" step is categorized by one that went above {driftTolerance}. Sort them by
        // distance to middle element (so we start with the middle elements first and expand to endpoints)
        auto badStepIndices = std::list<int32_t>();
        for (auto i = 0; i < N; ++i) {
            if (xyDrift[i] > driftTolerance) {
                badStepIndices.push_back(i);
            }
        }
        std::sort(badStepIndices.begin(), badStepIndices.end(), [N](int32_t a, int32_t b) {
            auto mid = N / 2;
            return ((std::abs(a - mid) < std::abs(b - mid)) ? a : b);
        });

        // Iterate over the bad step list, settling each particle
        auto iterationCount = 0;
        while (!badStepIndices.empty() && iterationCount < maxIterations) {
            // Element we're operating on this iteration
            auto elem = badStepIndices.front();
            badStepIndices.pop_front();

            // Get indices of neighbors
            auto neighborIndices = _getNeighborIndices(elem, neighborhoodRadius);
            if (neighborIndices.size() < neighborhoodRadius * 2) {
                badStepIndices.push_back(elem);
                continue;
            }

            // Directions and drifts of neighbors
            auto neighborDirections = std::vector<int8_t>(neighborIndices.size());
            auto neighborDrift = std::vector<double>(neighborIndices.size());
            std::transform(neighborIndices.begin(), neighborIndices.end(), std::back_inserter(neighborDirections),
                           [predictedDirections](int32_t i) { return predictedDirections[i]; });
            std::transform(neighborIndices.begin(), neighborIndices.end(), std::back_inserter(neighborDrift),
                           [xyDrift](int32_t i) { return xyDrift[i]; });

            // Get the majority direction and maximum drift value from neighbors
            auto majorityDirection = 0;
            auto directionSum = std::accumulate(neighborDirections.begin(), neighborDirections.end(), 0);
            if (directionSum > 0) {
                majorityDirection = Direction::kRight;
            } else if (directionSum < 0) {
                majorityDirection = Direction::kLeft;
            } else {
                // print something indicating no consensus from neighbors. Could do a couple things here:
                // 1. Expand neighborhood boundaries, retry
                // 2. Go with some default behavior
                // 3. ???
            }
            auto maxDrift = *std::max_element(neighborDrift.begin(), neighborDrift.end());

            // Restep this particle using new constraints
            cv::Vec3f newPosition;
            Direction newDirection;
            std::tie(predictedDirections[i], predictedPositions[i]) = currentChain_.step(i, majorityDirection, maxDrift);

            iterationCount++;
        }

        // Push old positions back into chainmesh
        mesh_.addChain(currentChain_);
        currentChain_.setNewPositions(predictedPositions);
    }
}

std::vector<int32_t>
LocalResliceSegmentation::_getNeighborIndices(const int32_t index, const int32_t radius)
{
    // Fill nbors with the neighboring indices from each side of {index}
    auto nbors = std::vector<int32_t>(radius * 2);
    auto nborCount = 0;
    auto nborOffset = 0;
    while (nborCount < radius * 2 && index + nborOffset < currentChain_.size() && index - nborOffset > 0) {
        nbors.push_back(index + nborOffset);
        nbors.push_back(index - nborOffset);
        nborCount += 2;
        nborOffset++;
    }

    // If we couldn't add neighbors fully on one side of the index, then fill it in with more neighbors from the other
    // side
    if (nborCount < radius * 2) {
        if (index + nborOffset >= currentChain_.size()) {
            while (nborCount < radius * 2 && index - nborOffset > 0) {
                nbors.push_back(index - nborOffset);
                nborOffset++;
                nborCount++;
            }
        } else if (index - nborOffset <= 0) {
            while (nborCount < radius * 2 && index + nborOffset < currentChain_.size()) {
                nbors.push_back(index + nborOffset);
                nborOffset++;
                nborCount++;
            }
        }
    }

    return nbors;
}