#include <list>
#include <tuple>
#include <numeric>

#include "localResliceParticleSim.h"


using namespace volcart::segmentation;

LocalResliceSegmentation::LocalResliceSegmentation(VolumePkg& pkg) :
        pkg_(pkg), startIndex_(0), endIndex_(0) { }

pcl::PointCloud<pcl::PointXYZRGB>
LocalResliceSegmentation::segmentLayer(const bool showVisualization,
                                       const double driftTolerance,
                                       const int32_t startIndex,
                                       const int32_t endIndex,
                                       const int32_t neighborhoodRadius,
                                       const int32_t stepsBeforeReslice,
                                       const int32_t stepNumLayers,
                                       const int32_t maxIterations)
{
    // The main chain that we'll be mutating on each iteration
    auto currentChain = Chain(pkg_, startIndex);

    startIndex_ = startIndex;
    // Account for zero-indexing and slices lost in calculating normal vector
    endIndex_ = pkg_.getNumberOfSlices() - 3;
    if (endIndex != kDefaultEndIndex) {
        endIndex_ = endIndex;
    }

    // ChainMesh that holds the segment
    auto mesh = ChainMesh(currentChain.size(), endIndex - startIndex);
    mesh.addChain(currentChain);

    // Go through every iteration (from start to end index)
    auto sliceIndex = startIndex_;
    while (sliceIndex < endIndex_) {
		std::cout << "slice: " << sliceIndex << std::endl;
        // Get predicted directions and positions
        if (showVisualization) {
            currentChain.draw();
            cv::waitKey(0);
        }
        std::vector<Direction> predictedDirections;
        std::vector<cv::Vec3d> predictedPositions;
        std::tie(predictedDirections, predictedPositions) = currentChain.stepAll(stepNumLayers);
        auto N = int32_t(predictedPositions.size());

        // Get XY drift of newPositions from currentPositions
        auto xyDrift = std::vector<double>(N);
        for (auto i = 0; i < N; ++i) {
            auto p1 = cv::Vec2d(predictedPositions[i](VC_INDEX_X), predictedPositions[i](VC_INDEX_Y));
            auto p2 = cv::Vec2d(currentChain.at(i)(VC_INDEX_X), currentChain.at(i)(VC_INDEX_Y));
            xyDrift[i] = cv::norm(p1, p2);
        }

        // "Bad" step indices. A "bad" step is categorized by one that went above
        // {driftTolerance}. Sort them by distance to middle element (so we start
        // with the middle elements first and expand to endpoints)
        auto badStepIndices = std::list<int32_t>();
        for (auto i = 0; i < N; ++i) {
            if (xyDrift[i] > driftTolerance) {
				//std::cout << "badstep: " << i << std::endl;
                badStepIndices.push_back(i);
            }
        }
        badStepIndices.sort([N](int32_t a, int32_t b) {
            auto mid = N / 2;
            return std::abs(a - mid) < std::abs(b - mid);
        });

        // Iterate over the bad step list, settling each particle
        auto iterationCount = 0;
        while (!badStepIndices.empty() && iterationCount < maxIterations) {
            // Element we're operating on this iteration
            auto elem = badStepIndices.front();
            badStepIndices.pop_front();

            // Get indices of neighbors
            auto neighborIndices = _getNeighborIndices(
                    currentChain, badStepIndices, elem, neighborhoodRadius);
            if (int32_t(neighborIndices.size()) < neighborhoodRadius * 2 * kExceedsNeighborRatioRetry) {
                badStepIndices.push_back(elem);
                continue;
            }

            // Directions and drifts of neighbors
            auto neighborDirections = std::vector<int8_t>(neighborIndices.size());
            auto neighborDrift = std::vector<double>(neighborIndices.size());
            std::transform(neighborIndices.begin(), neighborIndices.end(),
                           std::back_inserter(neighborDirections),
                           [predictedDirections](int32_t i) { return predictedDirections[i]; });
            std::transform(neighborIndices.begin(), neighborIndices.end(),
                           std::back_inserter(neighborDrift),
                           [xyDrift](int32_t i) { return xyDrift[i]; });

            // Get the majority direction and maximum drift value from neighbors
            Direction majorityDirection = Direction::kNone;
            auto directionSum = std::accumulate(neighborDirections.begin(),
                                                neighborDirections.end(), 0);
            if (directionSum > 0) {
                majorityDirection = Direction::kRight;
            } else if (directionSum < 0) {
                majorityDirection = Direction::kLeft;
            } else {
                majorityDirection = Direction::kNone;
            }
            auto maxDrift = *std::max_element(neighborDrift.begin(),
                                              neighborDrift.end());

            // Restep this particle using new constraints
            std::tie(predictedDirections[elem], predictedPositions[elem]) =
                    currentChain.step(elem, stepNumLayers, majorityDirection, maxDrift);

            iterationCount++;
        }
        //std::cout << "Got through " << iterationCount << " iterations" << std::endl;

        // Push old positions back into chainmesh
        currentChain.setNewPositions(predictedPositions);
        mesh.addChain(currentChain);
        sliceIndex += stepNumLayers;
        currentChain.setZIndex(sliceIndex);
    }

    return mesh.exportAsPointCloud();
}

std::vector<int32_t>
LocalResliceSegmentation::_getNeighborIndices(
        const Chain c, const std::list<int32_t>& badIndices,
        const int32_t index, const int32_t radius)
{
    // Set up start and end indices, accounting for if the neighborhood goes
    // off the end of the chain
    int32_t start = index - radius;
    int32_t end   = index + radius;
    if (start < 0) {
        end += -start;
        start = 0;
    } else if (end >= c.size()) {
        start -= (end - c.size() + 1);
        end = c.size() - 1;
    }

    // Add neighbors, but only if they're not in the bad indices list
    auto nbors = std::vector<int32_t>();
    for (int32_t i = start; i <= end; ++i) {
		if (i == index) {
			continue;
		}
        auto loc = std::find(std::begin(badIndices), std::end(badIndices), i);
        if (loc == std::end(badIndices)) {
            nbors.push_back(i);
        }
    }

    return nbors;
}
