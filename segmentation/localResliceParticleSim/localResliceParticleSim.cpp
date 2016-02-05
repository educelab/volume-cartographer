#include <list>
#include <tuple>
#include "localResliceParticleSim.h"
#include "common.h"
#include "fittedcurve.h"
#include "normalizedintensitymap.h"

using namespace volcart::segmentation;

std::vector<double> deriv(const vec<Voxel>& vec, const int32_t hstep = 1);

template <typename T1, typename T2>
vec<std::pair<T1, T2>> zip(vec<T1> v1, vec<T2> v2);

pcl::PointCloud<pcl::PointXYZRGB> exportAsPCD(const vec<vec<Voxel>>& points);

vec<double> subtractAbs(const vec<double>& lhs, const vec<double>& rhs)
{
    assert(lhs.size() == rhs.size() && "lhs must be the same size as rhs");
    vec<double> res(lhs.size(), 0.0);
    for (uint32_t i = 0; i < lhs.size(); ++i) {
        res[i] = std::fabs(lhs[i] - rhs[i]);
    }
    return res;
}

double normL2(const vec<double>& lhs, const vec<double>& rhs)
{
    assert(lhs.size() == rhs.size() && "lhs must be the same size as rhs");
    double res = 0.0;
    for (uint32_t i = 0; i < lhs.size(); ++i) {
        res += (lhs[i] - rhs[i]) * (lhs[i] - rhs[i]);
    }
    return std::sqrt(res);
}

pcl::PointCloud<pcl::PointXYZRGB> LocalResliceSegmentation::segmentPath(
    const vec<Voxel>& initPath, const double resamplePerc,
    const int32_t startIndex, const int32_t endIndex, const int32_t numIters,
    const int32_t keepNumMaxima, const int32_t step)
{
    volcart::Volume vol = pkg_.volume();
    std::cout << "init size: " << initPath.size() << std::endl;
    // Collection to hold all positions
    vec<vec<Voxel>> points;
    points.reserve((endIndex - startIndex + 1) / step);

    // Resample incoming curve
    FittedCurve initCurve(initPath, startIndex);
    initCurve.resample(resamplePerc);
    vec<Voxel> currentVs = initCurve.resampledPoints();
    std::cout << "resampled size: " << currentVs.size() << std::endl;

    // Iterate over z-slices
    for (int32_t zIndex = startIndex; zIndex <= endIndex; zIndex += step) {
        // 0. Resample current positions so they are evenly spaced
        FittedCurve curve{currentVs, zIndex};
        currentVs = curve.resampledPoints();

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
                nextPositions.push_back(std::deque<Voxel>{
                    reslice.sliceToVoxelCoord({center.x, nextLayerIndex})});
                continue;
            }

            // Convert top N maxima to voxel positions
            std::deque<Voxel> maximaQueue;
            for (int32_t i = 0; i < keepNumMaxima; ++i) {
                maximaQueue.emplace_back(reslice.sliceToVoxelCoord(
                    {allMaxima[i].first, nextLayerIndex}));
            }
            nextPositions.push_back(maximaQueue);
        }

        // 2. Construct initial guess using top maxima for each next position
        vec<Voxel> nextVs;
        nextVs.reserve(currentVs.size());
        for (auto&& d : nextPositions) {
            nextVs.push_back(d.front());
            d.pop_front();
        }
        const vec<double> currentDeriv = FittedCurve(currentVs, zIndex).deriv();
        vec<double> nextDeriv = FittedCurve(nextVs, zIndex).deriv();

        // 3. Iterative greedy algorithm to find particle introducing largest
        // difference in derivatives and keep choosing different positions until
        // it minimizes it
        vec<int32_t> ignoreList;
        int32_t i = 0;
        while (i < numIters) {
            // Find index of maximum difference between derivatives
            const vec<double> diff = subtractAbs(currentDeriv, nextDeriv);
            vec<int32_t> indxs(diff.size());
            std::iota(std::begin(indxs), std::end(indxs), 0);
            auto pairs = zip(indxs, diff);
            std::sort(std::begin(pairs), std::end(pairs),
                      [](const std::pair<int32_t, double> p1,
                         const std::pair<int32_t, double> p2) {
                          return p1.second < p2.second;
                      });

            // Skip any particles that we've run out of candidate positions for
            auto it = std::find(std::begin(ignoreList), std::end(ignoreList),
                                pairs.back().second);
            while (it != std::end(ignoreList)) {
                pairs.pop_back();
            }
            int32_t maxDiffIdx = pairs.back().first;

            // Get next candidate voxel for that index and construct new vector
            vec<Voxel> combVs(currentVs.begin(), currentVs.end());
            auto& candidates = nextPositions[maxDiffIdx];
            if (candidates.size() > 0) {
                combVs[maxDiffIdx] = candidates.front();
                candidates.pop_front();
            } else {
                ignoreList.push_back(maxDiffIdx);
                continue;
            }

            // Evaluate new combination and compare to best so far, if better
            // setting the new positions
            vec<double> combDeriv = FittedCurve(combVs, zIndex).deriv();
            if (normL2(combDeriv, currentDeriv) <
                normL2(nextDeriv, currentDeriv)) {
                nextVs = combVs;
                nextDeriv = combDeriv;
            }
            ++i;
        }

        // Add next positions to
        currentVs = nextVs;
        points.push_back(nextVs);
    }

    return exportAsPCD(points);
}

template <typename T1, typename T2>
vec<std::pair<T1, T2>> zip(vec<T1> v1, vec<T2> v2)
{
    assert(v1.size() == v2.size() && "v1 and v2 must be the same size");
    vec<std::pair<T1, T2>> res;
    res.reserve(v1.size());
    for (int32_t i = 0; i < int32_t(v1.size()); ++i) {
        res.emplace_back(v1[i], v2[i]);
    }
    return res;
}

cv::Vec3d LocalResliceSegmentation::estimateNormalAtIndex(
    const FittedCurve& curve, const int32_t index)
{
    const Voxel currentVoxel = curve(index);
    const auto eigenPairs = pkg_.volume().eigenPairsAtIndex(
        currentVoxel(0), currentVoxel(1), currentVoxel(2), 3);
    const double exp0 = std::log10(eigenPairs[0].first);
    const double exp1 = std::log10(eigenPairs[1].first);
    if (std::abs(exp0 - exp1) > 2.0) {
        return eigenPairs[0].second;
    }
    const auto tangent = curve.derivAt(index, 3);
    return tangent.cross(cv::Vec3d{0, 0, 1});
}

pcl::PointCloud<pcl::PointXYZRGB> exportAsPCD(const vec<vec<Voxel>>& points)
{
    int32_t rows = points.size();
    int32_t cols = points[0].size();
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.reserve(cols * rows);

    // Set size. Since this is unordered (for now...) just set the width to be
    // the number of points and the height (by convention) is set to 1
    cloud.width = cols * rows;
    cloud.height = 1;

    for (int32_t i = 0; i < rows; ++i) {
        for (int32_t j = 0; j < cols; ++j) {
            Voxel v = points[i][j];
            pcl::PointXYZRGB p;
            p.x = v(0);
            p.y = v(1);
            p.z = v(2);
            p.r = 0xFF;
            p.g = 0xFF;
            p.b = 0xFF;
            cloud.push_back(p);
        }
    }
    return cloud;
}

/*
pcl::PointCloud<pcl::PointXYZRGB> exportAsPCD(const vec<vec<Voxel>>& points)
{
    int32_t rows = points.size();
    int32_t cols = points[0].size();
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.reserve(rows * cols);

    // Set size. Since this is unordered (for now...) just set the width to be
    // the number of points and the height (by convention) is set to 1
    cloud.width = rows * cols;
    cloud.height = 1;

    for (int32_t i = 0; i < rows; ++i) {
        for (int32_t j = 0; j < cols; ++j) {
            pcl::PointXYZRGB p;
            p.x = points[i][j](0);
            p.y = points[i][j](1);
            p.z = points[i][j](2);
            p.r = 0xFF;
            p.g = 0xFF;
            p.b = 0xFF;
            cloud.push_back(p);
        }
    }
    return cloud;
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
*/
