#include <list>
#include <tuple>
#include <numeric>
#include <memory>
#include <random>

#include "localResliceParticleSim.h"
#include "common.h"

using namespace volcart::segmentation;

using ScalarVector = std::vector<double>;
std::vector<double> deriv(const VoxelVec& vec, const int32_t hstep = 1);

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

pcl::PointCloud<pcl::PointXYZRGB> segmentPath(const vec<Voxel>& initPath,
                                              const double resamplePerc,
                                              const int32_t startIndex,
                                              const int32_t endIndex,
                                              const int32_t keepNumMaxima,
                                              const int32_t step)
{
    // Current voxels
    vec<Voxel> currentVs = initPath;

    // Collection to hold all positions
    vec<vec<Voxel>> points;
    vec.reserve((endIndex - startIndex + 1) / step);

    // Iterate over z-slices
    for (int32_t zIndex = startIndex; zIndex <= endIndex; zIndex += step) {
        // Get parameterized rep of the incoming curve and resample it
        std::cout << "initial size: " << currentVs.size() << std::endl;
        FittedCurve curve(currentVs, zIndex);
        curve.resample(resamplePerc);

        // 1. Generate all candidate positions for all particles
        vec<std::deque<Voxel>> nextPositions;
        nextPositions.reserve(curve.size());
        // XXX DEBUG
        vec<NormalizedIntensityMap> maps;
        // XXX DEBUG
        for (int32_t i = 0; i < curve.size(); ++i) {
            // Estimate normal and reslice along it
            const cv::Vec3d normal = estimateNormalAtIndex(curve, i);
            const auto reslice = vol.reslice(curve(i), normal, {0, 0, 1});
            const cv::Mat resliceIntensities = reslice.sliceData();

            // Make the intensity map `step` layers down from current position
            // and find the maxima
            const cv::Point2i center{resliceIntensities.cols / 2,
                                     resliceIntensities.rows / 2};
            const int32_t nextLayerIndex = center.y + step;
            const NormalizedIntensityMap map{
                resliceIntensities.row(nextLayerIndex)};
            maps.push_back(map);
            const auto allMaxima = map.sortedMaxima();

            // Handle case where there's no maxima - go straight down
            if (allMaxima.empty()) {
                nextPositions.emplace_back(
                    reslice.sliceToVoxelCoord({center.x, nextLayerIndex}));
                continue;
            }

            // Convert top N maxima to voxel positions
            std::deque<Voxel> maximaQueue;
            for (int32_t i = 0; i < keepNumMaxima; ++i) {
                maximaQueue.emplace_back(reslice.sliceToVoxelCoord(
                    {allMaxima[i].first, nextLayerIndex}));
            }
            nextPositions.push_back(voxelMaxima);
        }

        // 2. Construct initial guess using top maxima for each next position
        const vec<double> currentK = curve.estimateCurvature();
    }
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
    currentPos = downsample(currentPos, startIndex, 0.30);
    std::cout << "currentPos size: " << currentPos.size() << std::endl;

    // ChainMesh that will hold all positions
    ChainMesh mesh(currentPos.size(),
                   (endIndex - startIndex + 1) / stepNumLayers);

    for (int32_t zIndex = startIndex; zIndex <= endIndex;
         zIndex += stepNumLayers) {
        Chain chain(pkg_, currentPos, zIndex, false, true);

        auto choiceSpace = chain.stepAll(stepNumLayers, keepNumMaxima);

        std::cout << "slice: " << zIndex << std::endl;
        if (showVisualization) {
            chain.draw();
            cv::waitKey(0);
        }

        // Get the first set of positions (all the 'best' positions independent
        // of neighborhood constraints)
        VoxelVec bestPos;
        bestPos.reserve(currentPos.size());
        for (auto& v : choiceSpace) {
            bestPos.push_back(v.front());
            v.pop_front();
        }

        ScalarVector initDerivs = chain.curve().deriv(3);
        ScalarVector bestPosDerivs = deriv(bestPos, 3);
        double minDerivativeDiff = normL2(bestPosDerivs, initDerivs);

        // Greedy algorithm to iteratively re-step the particle that introduces
        // the largest difference in the derivative
        for (int32_t i = 0; i < numIters; ++i) {
            // std::cout << "i: " << i << std::endl;
            // std::cout << "mindiff: " << minDerivativeDiff << "\n";
            // 1. Calculate the index of the maximum derivative difference
            ScalarVector absDiff = subtractAbs(bestPosDerivs, initDerivs);
            int32_t maxDiffIdx =
                std::distance(absDiff.begin(),
                              std::max_element(absDiff.begin(), absDiff.end()));

            // 2. Get the next candidate voxel position for that index
            // std::cout << "maxDiffIdx: " << maxDiffIdx << std::endl;
            VoxelVec combinationPos(bestPos.begin(), bestPos.end());
            if (choiceSpace[maxDiffIdx].size() > 0) {
                combinationPos[maxDiffIdx] = choiceSpace[maxDiffIdx].front();
                choiceSpace[maxDiffIdx].pop_front();
            } else {
                // std::cerr << "ran out of candidate positions\n";
                break;
            }

            // 3. Re-evaluate derivatives of best positions and find absdiff
            ScalarVector combinationPosDerivs = deriv(combinationPos, 3);
            // std::cout << "combination: "
            //          << normL2(combinationPosDerivs, initDerivs) <<
            //          std::endl;
            if (normL2(combinationPosDerivs, initDerivs) < minDerivativeDiff) {
                // std::cout << "Found better position\n";
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

std::vector<double> deriv(const VoxelVec& vec, const int32_t hstep)
{
    FittedCurve<double> curve(vec);
    return curve.deriv(hstep);
}
