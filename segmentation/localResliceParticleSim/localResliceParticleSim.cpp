#include <list>
#include <tuple>
#include <limits>
#include <boost/filesystem.hpp>
#include "localResliceParticleSim.h"
#include "common.h"
#include "fittedcurve.h"
#include "intensitymap.h"

using namespace volcart::segmentation;
namespace fs = boost::filesystem;
using std::begin;
using std::end;

template <typename T1, typename T2>
vec<std::pair<T1, T2>> zip(vec<T1> v1, vec<T2> v2);

pcl::PointCloud<pcl::PointXYZRGB> exportAsPCD(const vec<vec<Voxel>>& points);

vec<double> squareDiff(const vec<Voxel>& src, const vec<Voxel>& target);

double curveEnergy(const vec<Voxel>& src, const vec<Voxel>& target);

pcl::PointCloud<pcl::PointXYZRGB> LocalResliceSegmentation::segmentPath(
    const vec<Voxel>& initPath, const double resamplePerc,
    const int32_t startIndex, const int32_t endIndex, const int32_t numIters,
    const int32_t keepNumMaxima, const int32_t step, const bool dumpVis,
    const bool visualize, const int32_t visIndex)
{
    volcart::Volume vol = pkg_.volume();
    std::cout << "init size: " << initPath.size() << std::endl;

    // Debug output information
    const fs::path outputDir("debugvis");
    if (dumpVis) {
        fs::create_directory(outputDir);
    }

    // Collection to hold all positions
    vec<vec<Voxel>> points;
    points.reserve((endIndex - startIndex + 1) / step);

    // Resample incoming curve
    auto currentVs = FittedCurve(initPath, startIndex).resample(resamplePerc);
    std::cout << "resampled size: " << currentVs.size() << std::endl;

    // Iterate over z-slices
    for (int32_t zIndex = startIndex;
         zIndex <= endIndex && zIndex < pkg_.getNumberOfSlices();
         zIndex += step) {
        std::cout << "slice: " << zIndex << std::endl;

        // Directory to dump vis
        std::stringstream ss;
        ss << std::setw(std::to_string(endIndex).size()) << std::setfill('0')
           << zIndex;
        const fs::path zIdxDir = outputDir / ss.str();
        if (dumpVis) {
            fs::create_directory(zIdxDir);
        }

        // 0. Resample current positions so they are evenly spaced
        FittedCurve curve{currentVs, zIndex};
        currentVs = curve.resample(seedPointsseedPointsseedPoints);

        // 1. Generate all candidate positions for all particles
        vec<std::deque<Voxel>> nextPositions;
        nextPositions.reserve(curve.size());
        // XXX DEBUG
        vec<IntensityMap> maps;
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
            IntensityMap map{resliceIntensities.row(nextLayerIndex)};
            maps.push_back(map);
            const auto allMaxima = map.sortedMaxima();

            // Handle case where there's no maxima - go straight down
            if (allMaxima.empty()) {
                nextPositions.push_back(std::deque<Voxel>{
                    reslice.sliceToVoxelCoord({center.x, nextLayerIndex})});
                continue;
            }

            // Don't dump IntensityMap until we know which position the
            // algorithm will choose.
            if (dumpVis) {
                cv::Mat chain = drawParticlesOnSlice(currentVs, zIndex, i);
                cv::Mat resliceMat = reslice.draw();
                std::stringstream ss;
                ss << std::setw(2) << std::setfill('0') << zIndex << "_"
                   << std::setw(2) << std::setfill('0') << i;
                const fs::path base = zIdxDir / ss.str();
                cv::imwrite(base.string() + "_chain.png", chain);
                cv::imwrite(base.string() + "_reslice.png", resliceMat);

                // Additionaly, visualize if necessary
                if (visualize && i == visIndex) {
                    cv::namedWindow("slice", cv::WINDOW_NORMAL);
                    cv::namedWindow("reslice", cv::WINDOW_NORMAL);
                    cv::namedWindow("intensity map", cv::WINDOW_NORMAL);
                    cv::imshow("slice", chain);
                    cv::imshow("reslice", resliceMat);
                    cv::imshow("intensity map", map.draw());
                }
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
        for (auto&& map : maps) {
            map.setFinalChosenMaximaIndex(0);
        }

        // 3. Iterative greedy algorithm to find particle introducing largest
        // difference in derivatives and keep choosing different positions until
        // it minimizes it
        vec<int32_t> ignoreList;
        vec<int32_t> indxs(currentVs.size());
        std::iota(begin(indxs), end(indxs), 0);
        int32_t i = 0;
        while (i < numIters) {
            // Find index of maximum difference between positions
            const vec<double> diff = squareDiff(nextVs, currentVs);
            auto pairs = zip(indxs, diff);
            std::sort(begin(pairs), end(pairs),
                      [](const std::pair<int32_t, double> p1,
                         const std::pair<int32_t, double> p2) {
                          return p1.second < p2.second;
                      });

            // Skip any particles that we've run out of candidate positions for
            while (true) {
                auto it = std::find(begin(ignoreList), end(ignoreList),
                                    pairs.back().first);
                if (it == end(ignoreList)) {
                    break;
                } else {
                    pairs.pop_back();
                }
            }

            // Safety check for if we decide to ignore all indices. If so, then
            // we're done iterating through because there are no more positions
            // to consider
            if (pairs.empty()) {
                break;
            }

            // Get next candidate voxel for that index and construct new vector
            int32_t maxDiffIdx = pairs.back().first;
            vec<Voxel> combVs(begin(nextVs), end(nextVs));
            auto& candidates = nextPositions[maxDiffIdx];
            if (candidates.size() > 0) {
                combVs[maxDiffIdx] = candidates.front();
                maps[maxDiffIdx].incrementFinalChosenMaximaIndex();
                candidates.pop_front();
            } else {
                ignoreList.push_back(maxDiffIdx);
                continue;
            }

            // Evaluate new combination and compare to best so far, if better
            // setting the new positions
            if (curveEnergy(combVs, currentVs) <
                curveEnergy(nextVs, currentVs)) {
                nextVs = combVs;
            }
            ++i;
        }
        std::cout << "final curve energy: " << curveEnergy(nextVs, currentVs)
                  << std::endl;

        // Dump the intensity maps
        if (dumpVis) {
            for (size_t i = 0; i < maps.size(); ++i) {
                std::stringstream ss;
                ss << std::setw(2) << std::setfill('0') << zIndex << "_"
                   << std::setw(2) << std::setfill('0') << i << "_map.png";
                const fs::path base = zIdxDir / ss.str();
                cv::imwrite(base.string(), maps[i].draw());
            }
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
    const auto eigenPairs = pkg_.volume().eigenPairsAt(
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

vec<double> squareDiff(const vec<Voxel>& src, const vec<Voxel>& target)
{
    assert(src.size() == target.size() &&
           "src and target must be the same size");
    vec<double> res;
    res.reserve(src.size());
    for (const auto p : zip(src, target)) {
        Voxel s, t;
        std::tie(s, t) = p;
        res.push_back(std::sqrt((s(0) - t(0)) * (s(0) - t(0)) +
                                (s(1) - t(1)) * (s(1) - t(1))));
    }
    return res;
}

double curveEnergy(const vec<Voxel>& src, const vec<Voxel>& target)
{
    assert(src.size() == target.size() &&
           "src and target must be the same size");
    double sum = 0;
    for (const auto p : zip(src, target)) {
        Voxel s, t;
        std::tie(s, t) = p;
        sum += (s(0) - t(0)) * (s(0) - t(0)) + (s(1) - t(1)) * (s(1) - t(1));
    }
    return std::sqrt(sum);
}

cv::Mat LocalResliceSegmentation::drawParticlesOnSlice(
    const vec<Voxel>& vs, const int32_t sliceIndex,
    const int32_t particleIndex) const
{
    auto pkgSlice = pkg_.volume().getSliceDataCopy(sliceIndex);
    pkgSlice.convertTo(pkgSlice, CV_8UC3,
                       1.0 / std::numeric_limits<uint8_t>::max());
    cv::cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (const auto v : vs) {
        cv::Point real{int32_t(v(0)), int32_t(v(1))};
        cv::circle(pkgSlice, real, 1, BGR_GREEN, -1);
    }
    const Voxel particle = vs[particleIndex];
    cv::circle(pkgSlice, {int32_t(particle(0)), int32_t(particle(1))}, 1,
               BGR_RED, -1);

    /*
    // Superimpose interpolated curve on window
    if (showSpline) {
        const int32_t n = 50;
        for (double sum = 0; sum <= 1; sum += 1.0 / (n - 1)) {
            cv::Point p(curve_.eval(sum));
            cv::circle(pkgSlice, p, 1, BGR_BLUE, -1);
        }
    }
    */

    return pkgSlice;
}
