#include <list>
#include <tuple>
#include <numeric>
#include <memory>

#include "localResliceParticleSim.h"
#include "common.h"


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
    auto currentChain = std::unique_ptr<Chain>(new Chain(pkg_, startIndex));

    startIndex_ = startIndex;
    // Account for zero-indexing and slices lost in calculating normal vector
    endIndex_ = pkg_.getNumberOfSlices() - 3;
    if (endIndex != kDefaultEndIndex) {
        endIndex_ = endIndex;
    }

    // ChainMesh that holds the segment
    auto mesh = ChainMesh(currentChain->size(), endIndex - startIndex);
    mesh.addChain(*currentChain);

    // Get current derivatives
    int32_t N = currentChain->size();
    std::vector<double> currentDerivatives(N);
    for (int32_t i = 0; i < N; ++i) {
        currentDerivatives.at(i) = fivePointStencil(i, currentChain->positions());
    }

    // Go through every iteration (from start to end index)
    for (int32_t sliceIndex = startIndex_;
         sliceIndex <= endIndex_;
         sliceIndex += stepNumLayers) {

		std::cout << "slice: " << sliceIndex << std::endl;

        // Get predicted directions and positions
        if (showVisualization) {
            currentChain->draw();
            cv::waitKey(0);
        }
        std::vector<VoxelVectorType> allPredictedPositions =
            currentChain->stepAll(stepNumLayers);

        // Let's start picking new positions from the middle to the outside. Sort
        // indices based on distance from the center
        std::vector<int32_t> particleIndices(N);
        std::iota(particleIndices.begin(), particleIndices.end(), 0);
        std::sort(particleIndices.begin(), particleIndices.end(),
            [N](int32_t a, int32_t b) {
                int32_t mid = N / 2;
                return std::abs(a - mid) < std::abs(b - mid);
            });

        // Initialize current positions to first predicted (and likely best) position
        // in all positions structure
        // XXX Does this do move constructor or copy constructor?
        VoxelVectorType currentPositions(allPredictedPositions.at(0));
        VoxelVectorType minPositions(allPredictedPositions.at(0));

        // Exhaustive search for minimal change in chain derivative over all
        // combinations of predicted positions
        // XXX Make this faster by doing numerical optimization/gradient descent
        std::vector<double> currentDerivatives(N);
        std::vector<double> minDerivatives(N, std::numeric_limits<double>::max());
        for (int32_t i = 0; i < N; ++i) {
            for (int32_t j = 0; allPredictedPositions.at(i).size(); ++j) {
                currentPositions.at(i) = allPredictedPositions.at(i).at(j);
            }
        }



        /*
        // Get XY drift of newPositions from currentPositions
        auto xyDrift = std::vector<double>(N);
        for (auto i = 0; i < N; ++i) {
            auto p1 = cv::Vec2d(predictedPositions[i](VC_INDEX_X), predictedPositions[i](VC_INDEX_Y));
            auto p2 = cv::Vec2d(currentChain->at(i)(VC_INDEX_X), currentChain->at(i)(VC_INDEX_Y));
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
                    currentChain->step(elem, stepNumLayers, majorityDirection, maxDrift);

            iterationCount++;
        }
        //std::cout << "Got through " << iterationCount << " iterations" << std::endl;
        */

        // Push old positions back into chainmesh
        mesh.addPositions(minPositions);

        // Make a new chain for the next positions
        currentChain = std::unique_ptr<Chain>(new Chain(pkg_, sliceIndex));
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

// Uses the five point stencil equation as given here:
// https://en.wikipedia.org/wiki/Five-point_stencil
double LocalResliceSegmentation::fivePointStencil(
		const uint32_t center, const VoxelVectorType& ps) const
{

    // Average difference in x dimension across entire chain
    const double avgXDiff = std::accumulate(ps.begin(), ps.end(), 0,
            [](double sum, cv::Vec3d p) {
				return sum + p(VC_INDEX_X);
			}) / ps.size();

    // Fit curve to new predicted ps.
    FittedCurve<> f;
    decltype(f)::PointVectorType newPoints(ps.size());
    for (auto& p : ps) {
        newPoints.emplace_back(p(VC_INDEX_X), p(VC_INDEX_Y));
    }
    f.fitPoints(newPoints);

    // Take care of any out of bounds accesses
    double twoBefore, oneBefore, oneAfter, twoAfter;
    if (center == 0) {
        twoBefore = f.at(ps.at(0)(VC_INDEX_X) - 2 * avgXDiff);
        oneBefore = f.at(ps.at(0)(VC_INDEX_X) - avgXDiff);
        oneAfter  = ps.at(center + 1)(VC_INDEX_Y);
        twoAfter  = ps.at(center + 2)(VC_INDEX_Y);
    } else if (center == 1) {
        twoBefore = f.at(ps.at(0)(VC_INDEX_X) - avgXDiff);
        oneBefore = ps.at(0)(VC_INDEX_Y);
        oneAfter  = ps.at(center + 1)(VC_INDEX_Y);
        twoAfter  = ps.at(center + 2)(VC_INDEX_Y);
    } else if (center == ps.size() - 2) {
        twoBefore = ps.at(center - 2)(VC_INDEX_Y);
        oneBefore = ps.at(center - 1)(VC_INDEX_Y);
        oneAfter  = ps.at(ps.size() - 1)(VC_INDEX_Y);
        twoAfter  = f.at(ps.at(ps.size() - 1)(VC_INDEX_X) + avgXDiff);
    } else if (center == ps.size() - 1) {
        twoBefore = ps.at(center - 2)(VC_INDEX_Y);
        oneBefore = ps.at(center - 1)(VC_INDEX_Y);
        oneAfter  = f.at(ps.at(ps.size() - 1)(VC_INDEX_X) + avgXDiff);
        twoAfter  = f.at(ps.at(ps.size() - 1)(VC_INDEX_X) + 2 * avgXDiff);
    } else {
        twoBefore = ps.at(center - 2)(VC_INDEX_Y);
        oneBefore = ps.at(center - 1)(VC_INDEX_Y);
        oneAfter  = ps.at(center + 1)(VC_INDEX_Y);
        twoAfter  = ps.at(center + 2)(VC_INDEX_Y);
    }

    return (-twoAfter + 8 * oneAfter - 8 * oneBefore + twoBefore) / (12 * avgXDiff);
}
