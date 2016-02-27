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
std::vector<std::pair<T1, T2>> zip(const std::vector<T1>& v1,
                                   const std::vector<T2>& v2);

pcl::PointCloud<pcl::PointXYZRGB> exportAsPCD(
    const std::vector<std::vector<Voxel>>& points);

std::vector<double> squareDiff(const std::vector<Voxel>& v1,
                               const std::vector<Voxel>& v2);

std::vector<double> absDiff(const std::vector<double>& v1,
                            const std::vector<double>& v2);

std::vector<double> adjacentParticleDiff(const std::vector<Pixel>& vs);

double squareDiff(const std::vector<double>& v1, const std::vector<double>& v2);

double localInternalEnergy(int32_t index,
                           int32_t windowSize,
                           const FittedCurve& current,
                           const FittedCurve& next);

// Applies localInternalEnergy operator across entirety of both curves
double internalEnergy(const FittedCurve& current,
                      const FittedCurve& next,
                      double k1,
                      double k2);

double energyMetric(const FittedCurve& current,
                    const FittedCurve& next,
                    double alpha,
                    double beta,
                    double gama,
                    double k1,
                    double k2);

double globalEnergy(const FittedCurve& current, const FittedCurve& next);

pcl::PointCloud<pcl::PointXYZRGB> LocalResliceSegmentation::segmentPath(
    const std::vector<Voxel>& initPath,
    double resamplePerc,
    int32_t startIndex,
    int32_t endIndex,
    int32_t numIters,
    int32_t step,
    double alpha,
    double beta,
    double gama,
    double k1,
    double k2,
    int32_t peakDistanceWeight,
    bool dumpVis,
    bool visualize,
    int32_t visIndex)
{
    volcart::Volume vol = pkg_.volume();  // Debug output information
    const fs::path outputDir("debugvis");
    if (dumpVis) {
        fs::create_directory(outputDir);
    }

    // Collection to hold all positions
    std::vector<std::vector<Voxel>> points;
    points.reserve((endIndex - startIndex + 1) / step);

    // Resample incoming currentCurve
    auto currentVs = FittedCurve(initPath, startIndex).resample(resamplePerc);

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

        ////////////////////////////////////////////////////////////////////////////////
        // 0. Resample current positions so they are evenly spaced
        FittedCurve currentCurve{currentVs, zIndex};
        currentVs = currentCurve.seedPoints();

        ////////////////////////////////////////////////////////////////////////////////
        // 1. Generate all candidate positions for all particles
        std::vector<std::deque<Voxel>> nextPositions;
        nextPositions.reserve(currentCurve.size());
        // XXX DEBUG
        std::vector<IntensityMap> maps;
        // XXX DEBUG
        for (int32_t i = 0; i < currentCurve.size(); ++i) {
            // Estimate normal and reslice along it
            const cv::Vec3d normal = estimateNormalAtIndex(currentCurve, i);
            const auto reslice =
                vol.reslice(currentCurve(i), normal, {0, 0, 1}, 32, 32);
            const cv::Mat resliceIntensities = reslice.sliceData();

            // Make the intensity map `step` layers down from current position
            // and find the maxima
            const cv::Point2i center{resliceIntensities.cols / 2,
                                     resliceIntensities.rows / 2};
            const int32_t nextLayerIndex = center.y + step;
            IntensityMap map(resliceIntensities, step, peakDistanceWeight);
            const auto allMaxima = map.sortedMaxima();
            maps.push_back(map);

            // Handle case where there's no maxima - go straight down
            if (allMaxima.empty()) {
                nextPositions.push_back(
                    std::deque<Voxel>{reslice.sliceToVoxelCoord<int32_t>(
                        {center.x, nextLayerIndex})});
                continue;
            }

            // Don't dump IntensityMap until we know which position the
            // algorithm will choose.
            if (dumpVis) {
                cv::Mat chain = drawParticlesOnSlice(currentCurve, zIndex, i);
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

            // Convert maxima to voxel positions
            std::deque<Voxel> maximaQueue;
            for (const auto maxima : allMaxima) {
                maximaQueue.emplace_back(reslice.sliceToVoxelCoord<double>(
                    {maxima.first, nextLayerIndex}));
            }
            nextPositions.push_back(maximaQueue);
        }

        ////////////////////////////////////////////////////////////////////////////////
        // 2. Construct initial guess using top maxima for each next position
        std::vector<Voxel> nextVs;
        nextVs.reserve(currentVs.size());
        for (int32_t i = 0; i < int32_t(nextPositions.size()); ++i) {
            nextVs.push_back(nextPositions[i].front());
            maps[i].setChosenMaximaIndex(0);
        }
        FittedCurve nextCurve(nextVs, zIndex + 1);
        double minEnergy =
            energyMetric(currentCurve, nextCurve, alpha, beta, gama, k1, k2);
        double oldEnergy = std::numeric_limits<double>::max();
        double prevEnergyDiff = 0;
        double currEnergyDiff = oldEnergy - minEnergy;

        ////////////////////////////////////////////////////////////////////////////////
        // 3. Optimize
        std::vector<int32_t> indices(currentVs.size());
        std::iota(begin(indices), end(indices), 0);

        // - Go until either some hard limit or change in energy is minimal
        int32_t n = 0;
        bool energyChanged = false;
        while (n++ < numIters) {

            // Break condition for if we've reached a local minimum
            if (prevEnergyDiff - currEnergyDiff == 0 && !energyChanged) {
                break;
            }
            prevEnergyDiff = oldEnergy - minEnergy;

            // - Sort paired index-Voxel in increasing local internal energy
            auto pairs = zip(indices, squareDiff(currentVs, nextVs));
            std::sort(begin(pairs), end(pairs),
                      [](std::pair<int32_t, double> p1,
                         std::pair<int32_t, double> p2) {
                          return p1.second < p2.second;
                      });

            // - Go through the sorted list in reverse order, optimizing each
            // particle. If we find an optimum, then start over with the new
            // optimal positions. Do this until convergence or we hit a cap on
            // number of iterations.
            while (!pairs.empty()) {
                int32_t maxDiffIdx;
                double _;
                std::tie(maxDiffIdx, _) = pairs.back();
                pairs.pop_back();

                // Go through each combination for the maximal difference
                // particle, iterate until you find a new optimum or don't find
                // anything.
                for (uint32_t c = 0; c < nextPositions[maxDiffIdx].size();
                     ++c) {
                    std::vector<Voxel> combVs(begin(nextVs), end(nextVs));
                    combVs[maxDiffIdx] = nextPositions[maxDiffIdx][c];
                    FittedCurve combCurve(combVs, zIndex + 1);

                    const double newEnergy = energyMetric(
                        currentCurve, combCurve, alpha, beta, gama, k1, k2);

                    // Found a new optimum?
                    if (newEnergy < minEnergy) {
                        maps[maxDiffIdx].setChosenMaximaIndex(c);
                        nextVs = combVs;
                        nextCurve = combCurve;

                        // Shift energies back for comparison at the top
                        oldEnergy = minEnergy;
                        minEnergy = newEnergy;
                        currEnergyDiff = oldEnergy - minEnergy;
                        energyChanged = true;

                        if (visualize) {
                            cv::namedWindow("optimize chain",
                                            cv::WINDOW_NORMAL);
                            cv::imshow("optimize chain",
                                       drawParticlesOnSlice(combCurve, zIndex,
                                                            maxDiffIdx, false));
                            cv::waitKey(0);
                        }
                    }
                }

                // If we changed energy, then go back to the beginning and start
                // this whole process over to re-evaluate the new next curve
                if (energyChanged) {
                    energyChanged = false;
                    break;
                }
            }
        }

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

    ////////////////////////////////////////////////////////////////////////////////
    // 4. Output final mesh
    return exportAsPCD(points);
}

template <typename T1, typename T2>
std::vector<std::pair<T1, T2>> zip(const std::vector<T1>& v1,
                                   const std::vector<T2>& v2)
{
    assert(v1.size() == v2.size() && "v1 and v2 must be the same size");
    std::vector<std::pair<T1, T2>> res;
    res.reserve(v1.size());
    for (int32_t i = 0; i < int32_t(v1.size()); ++i) {
        res.push_back(std::make_pair(v1[i], v2[i]));
    }
    return res;
}

cv::Vec3d LocalResliceSegmentation::estimateNormalAtIndex(
    const FittedCurve& currentCurve, int32_t index)
{
    /*
    const Voxel currentVoxel = currentCurve(index);
    const auto eigenPairs = pkg_.volume().eigenPairsAt(
        currentVoxel(0), currentVoxel(1), currentVoxel(2), 3);
    const double exp0 = std::log10(eigenPairs[0].first);
    const double exp1 = std::log10(eigenPairs[1].first);
    if (std::abs(exp0 - exp1) > 2.0) {
        return eigenPairs[0].second;
    }
    */
    const auto tan2d = d1At(currentCurve.points(), index, 3);
    const Voxel tan3d{tan2d(0), tan2d(1), currentCurve(index)(2)};
    return tan3d.cross(cv::Vec3d{0, 0, 1});
}

pcl::PointCloud<pcl::PointXYZRGB> exportAsPCD(
    const std::vector<std::vector<Voxel>>& points)
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

std::vector<double> absDiff(const std::vector<double>& v1,
                            const std::vector<double>& v2)
{
    assert(v1.size() == v2.size() && "v1 must be same size as v2");
    std::vector<double> res;
    res.reserve(v1.size());
    const auto zipped = zip(v1, v2);
    std::transform(std::begin(zipped), std::end(zipped),
                   std::back_inserter(res),
                   [](const std::pair<double, double> p) {
                       return std::fabs(p.first - p.second);
                   });
    return res;
}

std::vector<double> squareDiff(const std::vector<Voxel>& v1,
                               const std::vector<Voxel>& v2)
{
    assert(v1.size() == v2.size() && "src and target must be the same size");
    std::vector<double> res;
    res.reserve(v1.size());
    const auto zipped = zip(v1, v2);
    std::transform(
        std::begin(zipped), std::end(zipped), std::back_inserter(res),
        [](const std::pair<Voxel, Voxel> p) {
            return std::sqrt(
                (p.first(0) - p.second(0)) * (p.first(0) - p.second(0)) +
                (p.first(1) - p.second(1)) * (p.first(1) - p.second(1)));
        });
    return res;
}

double squareDiff(const std::vector<double>& v1, const std::vector<double>& v2)
{
    assert(v1.size() == v2.size() && "v1 and v2 must be the same size");
    double res = 0;
    for (uint32_t i = 0; i < v1.size(); ++i) {
        res += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }
    return std::sqrt(res);
}

// Internal energy - how much did the curvature change between the two
// curves?
double localInternalEnergy(int32_t index,
                           int32_t windowSize,
                           const FittedCurve& current,
                           const FittedCurve& next)
{
    double internalEnergy = 0;
    const int32_t windowRadius = windowSize / 2;
    auto kCurrent = current.curvature();
    auto kNext = next.curvature();
    int32_t windowCount = 0;
    for (int32_t i = index - windowRadius; i <= index + windowRadius; ++i) {
        if (i < 0 || i >= current.size()) {
            continue;
        }
        internalEnergy += (kNext[i] - kCurrent[i]) * (kNext[i] - kCurrent[i]);
        windowCount++;
    }
    return internalEnergy / windowCount;
}

double internalEnergy(const FittedCurve& current,
                      const FittedCurve& next,
                      double k1,
                      double k2)
{
    assert(current.size() == next.size() &&
           "current and next curves must be same size");

    auto currentPoints = current.points();
    auto nextPoints = next.points();

    auto d1current = d1(currentPoints);
    auto d1next = d1(nextPoints);
    auto d2current = d2(currentPoints);
    auto d2next = d2(nextPoints);

    double intE = 0;
    for (int32_t i = 0; i < int32_t(currentPoints.size()); ++i) {
        intE += k1 * ((std::pow(d1next[i](0) - d1current[i](0), 2) +
                       std::pow(d1next[i](1) - d1current[i](1), 2))) +
                k2 * ((std::pow(d2next[i](0) - d2current[i](0), 2) +
                       std::pow(d2next[i](1) - d2current[i](1), 2)));
    }

    return std::sqrt(intE);
}

double tensionEnergy(const FittedCurve& current, const FittedCurve& next)
{
    const auto currentDiff = adjacentParticleDiff(current.points());
    const auto nextDiff = adjacentParticleDiff(next.points());
    return squareDiff(currentDiff, nextDiff);
}

double energyMetric(const FittedCurve& current,
                    const FittedCurve& next,
                    double alpha,
                    double beta,
                    double gama,
                    double k1,
                    double k2)
{
    const double global = globalEnergy(current, next);
    const double internal = internalEnergy(current, next, k1, k2);
    const double tension = tensionEnergy(current, next);
    /*
    std::cout << "global:   " << global << std::endl;
    std::cout << "internal: " << internal << std::endl;
    std::cout << "tension:  " << tension << std::endl;
    */
    return alpha * global + beta * internal + gama * tension;
}

// Global energy - how much does the next currentCurve look like the
// previous currentCurve?
double globalEnergy(const FittedCurve& current, const FittedCurve& next)
{
    double globalEnergy = 0;
    for (const auto p : zip(current.points(), next.points())) {
        Pixel s, t;
        std::tie(s, t) = p;
        globalEnergy +=
            (s(0) - t(0)) * (s(0) - t(0)) + (s(1) - t(1)) * (s(1) - t(1));
    }
    return std::sqrt(globalEnergy);
}

std::vector<double> adjacentParticleDiff(const std::vector<Pixel>& vs)
{
    // Standard L2 distance between XY components of Voxels
    auto pixelDiff = [](Pixel v1, Pixel v2) {
        return std::sqrt((v1(0) - v2(0)) * (v1(0) - v2(0)) +
                         (v1(1) - v2(1)) * (v1(1) - v2(1)));
    };

    std::vector<double> diffs(vs.size());

    // Special case for first and last elements. Since they don't have
    // neighbors, just double their distances to the elements next to them. This
    // might not be the best, but it's what works for now
    diffs.front() = 2 * pixelDiff(vs[0], vs[1]);
    diffs.back() = 2 * pixelDiff(vs.back(), vs.rbegin()[1]);

    // Handle the rest of the vector
    for (uint32_t i = 1; i < vs.size() - 1; ++i) {
        diffs[i] = pixelDiff(vs[i - 1], vs[i]) + pixelDiff(vs[i], vs[i + 1]);
    }

    return diffs;
}

cv::Mat LocalResliceSegmentation::drawParticlesOnSlice(const FittedCurve& curve,
                                                       int32_t sliceIndex,
                                                       int32_t particleIndex,
                                                       bool showSpline) const
{
    auto pkgSlice = pkg_.volume().getSliceDataCopy(sliceIndex);
    pkgSlice.convertTo(pkgSlice, CV_8UC3,
                       1.0 / std::numeric_limits<uint8_t>::max());
    cv::cvtColor(pkgSlice, pkgSlice, CV_GRAY2BGR);

    // draw circles on the pkgSlice window for each point
    for (int32_t i = 0; i < curve.size(); ++i) {
        cv::Point real{int32_t(curve(i)(0)), int32_t(curve(i)(1))};
        cv::circle(pkgSlice, real, (showSpline ? 2 : 1), BGR_GREEN, -1);
    }
    const Voxel particle = curve(particleIndex);
    cv::circle(pkgSlice, {int32_t(particle(0)), int32_t(particle(1))},
               (showSpline ? 2 : 1), BGR_RED, -1);

    // Superimpose interpolated currentCurve on window
    if (showSpline) {
        const int32_t n = 100;
        for (double sum = 0; sum <= 1; sum += 1.0 / (n - 1)) {
            cv::Point p(curve.eval(sum));
            cv::circle(pkgSlice, p, 1, BGR_BLUE, -1);
        }
    }

    return pkgSlice;
}
