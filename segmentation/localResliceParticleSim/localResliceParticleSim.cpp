#include <list>
#include <tuple>
#include <numeric>
#include <memory>
#include <random>

#include "localResliceParticleSim.h"
#include "common.h"

using namespace volcart::segmentation;

using ScalarVector = std::vector<double>;
std::vector<double> deriv(const VoxelVec& vec);

ScalarVector subtractAbs(const ScalarVector& lhs, const ScalarVector& rhs)
{
    assert(lhs.size() == rhs.size() && "lhs must be the same size as rhs");
    ScalarVector res(lhs.size(), 0.0);
    for (uint32_t i = 0; i < lhs.size(); ++i) {
        res[i] = std::fabs(lhs[i] - rhs[i]);
    }
    return res;
}

double normL2(const ScalarVector& lhs, const ScalarVector& rhs)
{
    assert(lhs.size() == rhs.size() && "lhs must be the same size as rhs");
    double res = 0.0;
    for (uint32_t i = 0; i < lhs.size(); ++i) {
        res += (lhs[i] - rhs[i]) * (lhs[i] - rhs[i]);
    }
    return std::sqrt(res);
}

VoxelVec downsample(const VoxelVec& voxels, int32_t zIndex,
                    double keepPerc = 0.40)
{
    FittedCurve<double> curve(voxels);
    int32_t newSize = std::round(voxels.size() * keepPerc);
    curve.resample(newSize);
    auto resampled = curve.resampledPoints();
    VoxelVec vResampled;
    vResampled.reserve(newSize);
    for (auto v : resampled) {
        vResampled.emplace_back(v(0), v(1), zIndex);
    }
    return vResampled;
}

LocalResliceSegmentation::LocalResliceSegmentation(VolumePkg& pkg) : pkg_(pkg)
{
}

pcl::PointCloud<pcl::PointXYZRGB> LocalResliceSegmentation::segmentLayer(
    const bool showVisualization, const int32_t startIndex,
    const int32_t endIndex, const int32_t stepNumLayers,
    const double derivativeTolerance, const int32_t keepNumMaxima,
    const int32_t numIters)
{
    // Starting positions
    VoxelVec currentPos;
    auto path = pkg_.openCloud();
    for (const auto& p : *path) {
        currentPos.emplace_back(p.x, p.y, p.z);
    }
    currentPos = downsample(currentPos, startIndex, 0.40);
    std::cout << "currentPos size: " << currentPos.size() << std::endl;

    // ChainMesh that will hold all positions
    ChainMesh mesh(currentPos.size(), endIndex - startIndex);

    for (int32_t zIndex = startIndex; zIndex <= endIndex;
         zIndex += stepNumLayers) {
        // First chain object
        Chain chain(pkg_, currentPos, zIndex);

        std::cout << "slice: " << zIndex << std::endl;
        if (showVisualization) {
            chain.draw();
            cv::waitKey(0);
        }

        auto choiceSpace = chain.stepAll(stepNumLayers, keepNumMaxima);

        // Get the first set of positions (all the 'best' positions independent
        // of neighborhood constraints)
        VoxelVec bestPos;
        bestPos.reserve(currentPos.size());
        for (auto& v : choiceSpace) {
            bestPos.push_back(v.front());
            v.pop_front();
        }

        ScalarVector initDerivs = chain.curve().deriv();
        ScalarVector bestPosDerivs = deriv(bestPos);
        double minDerivativeDiff = normL2(bestPosDerivs, initDerivs);

        // Greedy algorithm to iteratively re-step the particle that introduces
        // the largest difference in the derivative
        for (int32_t i = 0; i < numIters; ++i) {
            std::cout << "i: " << i << std::endl;
            std::cout << "mindiff: " << minDerivativeDiff << "\n";
            // 1. Calculate the index of the maximum derivative difference
            ScalarVector absDiff = subtractAbs(bestPosDerivs, initDerivs);
            int32_t maxDiffIdx =
                std::distance(absDiff.begin(),
                              std::max_element(absDiff.begin(), absDiff.end()));

            // 2. Get the next candidate voxel position for that index
            std::cout << "maxDiffIdx: " << maxDiffIdx << std::endl;
            VoxelVec combinationPos(bestPos.begin(), bestPos.end());
            if (choiceSpace[maxDiffIdx].size() > 0) {
                combinationPos[maxDiffIdx] = choiceSpace[maxDiffIdx].front();
                choiceSpace[maxDiffIdx].pop_front();
            } else {
                std::cerr << "ran out of candidate positions\n";
                break;
            }

            // 3. Re-evaluate derivatives of best positions and find absdiff
            ScalarVector combinationPosDerivs = deriv(combinationPos);
            std::cout << "combination: "
                      << normL2(combinationPosDerivs, initDerivs) << std::endl;
            if (normL2(combinationPosDerivs, initDerivs) < minDerivativeDiff) {
                bestPos = combinationPos;
                bestPosDerivs = combinationPosDerivs;
                minDerivativeDiff = normL2(combinationPosDerivs, initDerivs);
            }
        }

        // At the end, bestPos contains the best voxel positions
        mesh.addPositions(currentPos);
        currentPos = bestPos;
    }

    return mesh.exportAsPointCloud();
}

std::vector<double> deriv(const VoxelVec& vec)
{
    FittedCurve<double> curve(vec);
    return curve.deriv();
}
