#include <list>
#include <tuple>
#include <limits>
#include <boost/filesystem.hpp>
#include <boost/circular_buffer.hpp>
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

double energyMetric(
    const FittedCurve& curve, double alpha, double beta, double k1, double k2);

double tensionEnergy(const FittedCurve& curve);

double globalEnergy(const FittedCurve& current, const FittedCurve& next);

template <typename T>
std::vector<T> normalizeVector(const std::vector<T>& v,
                               T newMin = 0,
                               T newMax = 1)
{
    T min, max;
    auto p = std::minmax_element(begin(v), end(v));
    min = *p.first;
    max = *p.second;
    std::vector<T> norm_v;
    norm_v.reserve(v.size());

    // Normalization of [min, max] --> [-1, 1]
    std::transform(begin(v), end(v), std::back_inserter(norm_v),
                   [min, max, newMin, newMax](T t) {
                       return ((newMax - newMin) / (max - min)) * t +
                              ((newMin * max - min * newMax) / (max - min));
                   });
    return norm_v;
}

template <typename T, int32_t Len>
std::vector<cv::Vec<T, Len>> normalizeVector(
    const std::vector<cv::Vec<T, Len>> vs)
{
    std::vector<cv::Vec<T, Len>> new_vs;
    new_vs.reserve(vs.size());
    std::transform(begin(vs), end(vs), std::back_inserter(new_vs),
                   [](cv::Vec<T, Len> v) { return v / cv::norm(v); });
    return new_vs;
}

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
    std::cout << "incoming size: " << initPath.size() << std::endl;
    volcart::Volume vol = pkg_.volume();  // Debug output information
    const fs::path outputDir("debugvis");
    const fs::path wholeChainDir(outputDir / "whole_chain");
    if (dumpVis) {
        fs::create_directory(outputDir);
        fs::create_directory(wholeChainDir);
    }

    // Collection to hold all positions
    std::vector<std::vector<Voxel>> points;
    points.reserve((endIndex - startIndex + 1) / step);

    // Resample incoming currentCurve
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

        ////////////////////////////////////////////////////////////////////////////////
        // 0. Resample current positions so they are evenly spaced
        FittedCurve currentCurve{currentVs, zIndex};
        currentVs = currentCurve.seedPoints();

        // Dump entire curve for easy viewing
        if (dumpVis) {
            ss.str(std::string());
            ss << std::setw(std::to_string(endIndex).size())
               << std::setfill('0') << zIndex << "_chain.png";
            const auto wholeChainPath = wholeChainDir / ss.str();
            cv::imwrite(wholeChainPath.string(),
                        drawParticlesOnSlice(currentCurve, zIndex, 0));
        }

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
                const size_t nchars = std::to_string(endIndex).size();
                std::stringstream ss;
                ss << std::setw(nchars) << std::setfill('0') << zIndex << "_"
                   << std::setw(nchars) << std::setfill('0') << i;
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

        // Calculate energy of the current curve
        double minEnergy = energyMetric(nextCurve, alpha, beta, k1, k2);

        // Derivative of energy measure - keeps the previous three measurements
        // and will evaluate the central difference (when there's enough
        // measurements)
        boost::circular_buffer<double> dEnergy(3);
        dEnergy.push_back(minEnergy);

        ////////////////////////////////////////////////////////////////////////////////
        // 3. Optimize
        std::vector<int32_t> indices(currentVs.size());
        std::iota(begin(indices), end(indices), 0);

        // - Go until either some hard limit or change in energy is minimal
        int32_t n = 0;

    iters_start:
        while (n++ < numIters) {

            dEnergy.push_back(minEnergy);
            std::cout << "dEnergy: ";
            for (auto de : dEnergy) {
                std::cout << de << ", ";
            }
            std::cout << std::endl;

            bool energyChanged = false;
            // Break if our energy gradient is leveling off
            if (dEnergy.size() == 3 &&
                0.5 * (dEnergy[0] - dEnergy[2]) < kDefaultMinEnergyGradient) {
                break;
            }

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

                    // Found a new optimum?
                    double newE = energyMetric(combCurve, alpha, beta, k1, k2);
                    if (newE < minEnergy) {
                        energyChanged = true;
                        minEnergy = newE;
                        maps[maxDiffIdx].setChosenMaximaIndex(c);
                        nextVs = combVs;
                        nextCurve = combCurve;

                        if (visualize) {
                            cv::namedWindow("optimize chain",
                                            cv::WINDOW_NORMAL);
                            cv::imshow("optimize chain",
                                       drawParticlesOnSlice(combCurve, zIndex,
                                                            maxDiffIdx, false));
                            cv::waitKey(0);
                        }

                        goto iters_start;
                    }
                }
            }
        }

        // Dump the intensity maps
        if (dumpVis) {
            for (size_t i = 0; i < maps.size(); ++i) {
                std::stringstream ss;
                const size_t nchars = std::to_string(endIndex).size();
                ss << std::setw(nchars) << std::setfill('0') << zIndex << "_"
                   << std::setw(nchars) << std::setfill('0') << i << "_map.png";
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

double internalEnergy(const FittedCurve& curve, double k1, double k2)
{
    const auto currentPoints = curve.points();
    auto d1current = normalizeVector(d1(currentPoints));
    auto d2current = normalizeVector(d2(currentPoints));

    double intE = 0;
    for (auto p : zip(d1current, d2current)) {
        Pixel d1sq, d2sq;
        cv::pow(p.first, 2, d1sq);
        cv::pow(p.second, 2, d2sq);
        intE += k1 * cv::norm(d1sq) + k2 * cv::norm(d2sq);
    }

    return intE / (2 * curve.size());
}

double tensionEnergy(const FittedCurve& curve)
{
    auto diff = adjacentParticleDiff(curve.points());
    return std::accumulate(begin(diff), end(diff), 0.0) / curve.size();
}

double energyMetric(
    const FittedCurve& curve, double alpha, double beta, double k1, double k2)
{
    // const double global = globalEnergy(current, next);
    double internal = internalEnergy(curve, k1, k2);
    // double tension = tensionEnergy(curve);
    // std::cout << "global:   " << global << std::endl;
    // std::cout << "internal: " << internal << ",  tension: " << tension <<
    // std::endl;
    // return alpha * global + beta * internal + gama * tension;
    // return (tension + internal) / 2;
    // return alpha * tension + beta * internal;
    return (alpha + beta) * internal;
}

std::vector<double> adjacentParticleDiff(const std::vector<Pixel>& vs)
{
    std::vector<double> diffs(vs.size());

    // Special case for first and last elements. Since they don't have
    // neighbors, just double their distances to the elements next to them. This
    // might not be the best, but it's what works for now
    diffs.front() = 2 * cv::norm(vs[0], vs[1]);
    diffs.back() = 2 * cv::norm(vs.back(), vs.rbegin()[1]);

    // Handle the rest of the vector
    for (uint32_t i = 1; i < vs.size() - 1; ++i) {
        diffs[i] = cv::norm(vs[i - 1], vs[i]) + cv::norm(vs[i], vs[i + 1]);
    }

    return normalizeVector(diffs);
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
