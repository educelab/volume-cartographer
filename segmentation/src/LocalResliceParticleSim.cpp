#include <iomanip>
#include <limits>
#include <list>
#include <tuple>

#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/segmentation/lrps/Common.hpp"
#include "vc/segmentation/lrps/Derivative.hpp"
#include "vc/segmentation/lrps/EnergyMetrics.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"
#include "vc/segmentation/lrps/IntensityMap.hpp"
#include "vc/segmentation/lrps/LocalResliceParticleSim.hpp"

using namespace volcart::segmentation;
namespace fs = boost::filesystem;
using std::begin;
using std::end;

volcart::OrderedPointSet<cv::Vec3d> ExportAsPCD(
    const std::vector<std::vector<Voxel>>& points);

volcart::OrderedPointSet<cv::Vec3d> LocalResliceSegmentation::segmentPath(
    std::vector<cv::Vec3d> cloud,
    int startIndex,
    int endIndex,
    int numIters,
    int step,
    double alpha,
    double k1,
    double k2,
    double beta,
    double delta,
    int peakDistanceWeight,
    bool shouldIncludeMiddle,
    bool dumpVis,
    bool visualize)
{
    // Convert incoming cloud to voxel vector representation
    std::vector<Voxel> currentVs;
    currentVs.reserve(cloud.size());
    for (auto p : cloud) {
        currentVs.emplace_back(p[0], p[1], p[2]);
    }

    const volcart::Volume& vol = pkg_.volume();  // Debug output information

    // Check that incoming points are all within volume bounds. If not, then
    // return empty cloud back
    if (std::any_of(begin(currentVs), end(currentVs), [vol](auto v) {
            return !vol.isInBounds(v);
        })) {
        std::cerr << "[info]: one or more particles is outside volume bounds, "
                     "halting segmentation"
                  << std::endl;
        return volcart::OrderedPointSet<cv::Vec3d>();
    }

    const fs::path outputDir("debugvis");
    const fs::path wholeChainDir(outputDir / "whole_chain");
    if (dumpVis) {
        fs::create_directory(outputDir);
        fs::create_directory(wholeChainDir);
    }

    // Collection to hold all positions
    std::vector<std::vector<Voxel>> points;
    points.reserve((endIndex - startIndex + 1) / step);
    points.push_back(currentVs);

    // Iterate over z-slices
    for (int zIndex = startIndex;
         zIndex <= endIndex && zIndex < pkg_.getNumberOfSlices();
         zIndex += step) {
        // std::cout << "slice: " << zIndex << std::endl;

        // Directory to dump vis
        std::stringstream ss;
        ss << std::setw(std::to_string(endIndex).size()) << std::setfill('0')
           << zIndex;
        const fs::path zIdxDir = outputDir / ss.str();

        ////////////////////////////////////////////////////////////////////////////////
        // 0. Resample current positions so they are evenly spaced
        FittedCurve currentCurve(currentVs, zIndex);
        currentVs = currentCurve.evenlySpacePoints();

        // Dump entire curve for easy viewing
        if (dumpVis) {
            ss.str(std::string());
            ss << std::setw(std::to_string(endIndex).size())
               << std::setfill('0') << zIndex << "_chain.png";
            const auto wholeChainPath = wholeChainDir / ss.str();
            cv::imwrite(
                wholeChainPath.string(),
                draw_particle_on_slice_(currentCurve, zIndex, -1, true));
        }

        ////////////////////////////////////////////////////////////////////////////////
        // 1. Generate all candidate positions for all particles
        std::vector<std::deque<Voxel>> nextPositions;
        nextPositions.reserve(currentCurve.size());
        // XXX DEBUG
        std::vector<IntensityMap> maps;
        std::vector<Slice> reslices;
        maps.reserve(currentCurve.size());
        reslices.reserve(currentCurve.size());
        // XXX DEBUG
        for (int i = 0; i < int(currentCurve.size()); ++i) {
            // Estimate normal and reslice along it
            const cv::Vec3d normal = estimate_normal_at_index_(currentCurve, i);
            const auto reslice = vol.reslice(
                currentCurve(i), normal, {0, 0, 1}, resliceSize_, resliceSize_);
            reslices.push_back(reslice);
            auto resliceIntensities = reslice.sliceData();

            // Make the intensity map `step` layers down from current position
            // and find the maxima
            const cv::Point2i center{resliceIntensities.cols / 2,
                                     resliceIntensities.rows / 2};
            const int nextLayerIndex = center.y + step;
            IntensityMap map(
                resliceIntensities, step, peakDistanceWeight,
                shouldIncludeMiddle);
            const auto allMaxima = map.sortedMaxima();
            maps.push_back(map);

            // Handle case where there's no maxima - go straight down
            if (allMaxima.empty()) {
                nextPositions.emplace_back(
                    std::deque<Voxel>{reslice.sliceToVoxelCoord<int>(
                        {center.x, nextLayerIndex})});
                continue;
            }

            // Convert maxima to voxel positions
            std::deque<Voxel> maximaQueue;
            for (auto&& maxima : allMaxima) {
                maximaQueue.emplace_back(reslice.sliceToVoxelCoord<double>(
                    {maxima.first, nextLayerIndex}));
            }
            nextPositions.push_back(maximaQueue);
        }

        ////////////////////////////////////////////////////////////////////////////////
        // 2. Construct initial guess using top maxima for each next position
        std::vector<Voxel> nextVs;
        nextVs.reserve(currentVs.size());
        for (int i = 0; i < int(nextPositions.size()); ++i) {
            nextVs.push_back(nextPositions[i].front());
            maps[i].setChosenMaximaIndex(0);
        }
        FittedCurve nextCurve(nextVs, zIndex + 1);

        // Calculate energy of the current curve
        double minEnergy = std::numeric_limits<double>::max();

        // Derivative of energy measure - keeps the previous three measurements
        // and will evaluate the central difference (when there's enough
        // measurements)
        boost::circular_buffer<double> dEnergy(3);
        dEnergy.push_back(minEnergy);

        ////////////////////////////////////////////////////////////////////////////////
        // 3. Optimize
        std::vector<int> indices(currentVs.size());
        std::iota(begin(indices), end(indices), 0);

        // - Go until either some hard limit or change in energy is minimal
        int n = 0;

    iters_start:
        while (n++ < numIters) {

            // Break if our energy gradient is leveling off
            dEnergy.push_back(minEnergy);
            if (dEnergy.size() == 3 &&
                0.5 * (dEnergy[0] - dEnergy[2]) < DEFAULT_MIN_ENERGY_GRADIENT) {
                break;
            }

            // - Sort paired index-Voxel in increasing local internal energy
            auto pairs = Zip(indices, SquareDiff(currentVs, nextVs));
            std::sort(begin(pairs), end(pairs), [](auto p1, auto p2) {
                return p1.second < p2.second;
            });

            // - Go through the sorted list in reverse order, optimizing each
            // particle. If we find an optimum, then start over with the new
            // optimal positions. Do this until convergence or we hit a cap on
            // number of iterations.
            while (!pairs.empty()) {
                int maxDiffIdx;
                double _;
                std::tie(maxDiffIdx, _) = pairs.back();
                pairs.pop_back();

                // Go through each combination for the maximal difference
                // particle, iterate until you find a new optimum or don't find
                // anything.
                while (!nextPositions[maxDiffIdx].empty()) {
                    std::vector<Voxel> combVs(begin(nextVs), end(nextVs));
                    combVs[maxDiffIdx] = nextPositions[maxDiffIdx].front();
                    nextPositions[maxDiffIdx].pop_front();
                    FittedCurve combCurve(combVs, zIndex + 1);

                    // Found a new optimum?
                    double newE = EnergyMetrics::TotalEnergy(
                        combCurve, alpha, k1, k2, beta, delta);
                    if (newE < minEnergy) {
                        minEnergy = newE;
                        maps[maxDiffIdx].incrementMaximaIndex();
                        nextVs = combVs;
                        nextCurve = combCurve;
                    }
                }
                goto iters_start;
            }
        }

        ////////////////////////////////////////////////////////////////////////////////
        // 3. Clamp points that jumped too far back to a good (interpolated)
        // position. Do this by looking for places where the square of the
        // second derivative is large, and move them back. Tentatively, I'm only
        // going to move back points whose D2^2 evaluates to > 10.
        //
        // Currently, linear interpolation between the previous/next point is
        // used. This could be upgraded to some kind of other fit, possibly
        // cubic interpolation. The end points are linearly extrapolated from
        // their two closest neighbors.

        // Take initial second derivative
        auto secondDeriv = D2(nextVs);
        std::vector<double> normDeriv2(secondDeriv.size());
        std::transform(
            begin(secondDeriv) + 1, end(secondDeriv) - 1, begin(normDeriv2),
            [](auto d) { return cv::norm(d) * cv::norm(d); });

        // Don't resettle points at the beginning or end of the chain
        auto maxVal =
            std::max_element(begin(normDeriv2) + 1, end(normDeriv2) - 1);
        int settlingIters = 0;

        // Iterate until we move all out-of-place points back into place
        while (*maxVal > 10.0 && settlingIters++ < 100) {
            Voxel newPoint;
            int i = maxVal - begin(normDeriv2);
            Voxel diff = 0.5 * nextVs[size_t(i) + 1] - 0.5 * nextVs[i - 1];
            newPoint = nextVs[i - 1] + diff;
            nextVs[i] = newPoint;

            // Re-evaluate second derivative of new curve
            secondDeriv = D2(nextVs);
            std::transform(
                begin(secondDeriv), end(secondDeriv), begin(normDeriv2),
                [](auto d) { return cv::norm(d) * cv::norm(d); });

            // Don't resettle points at the beginning or end of the chain
            maxVal =
                std::max_element(begin(normDeriv2) + 1, end(normDeriv2) - 1);
        }

        // Check if any points in nextVs are outside volume boundaries. If so,
        // stop iterating and dump the resulting pointcloud.
        if (std::any_of(begin(nextVs), end(nextVs), [vol](auto v) {
                return !vol.isInBounds(v);
            })) {
            std::cout
                << "Stopping because segmentation is outside volume bounds"
                << std::endl;
            break;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // 4. Visualize if specified by user
        if (visualize) {
            // Since points can change due to 2nd deriv optimization after main
            // optimization, refit a curve and draw that
            FittedCurve newChain(nextVs, zIndex + 1);
            auto chain = draw_particle_on_slice_(newChain, zIndex + 1);
            cv::namedWindow("Next curve", cv::WINDOW_NORMAL);
            cv::imshow("Next curve", chain);
            cv::waitKey(0);
        }

        // Don't dump IntensityMap until we know which position the
        // algorithm will choose.
        if (dumpVis) {
            // Create output directory for this iter's output
            const size_t nchars = std::to_string(endIndex).size();
            std::stringstream iterDirSS;
            iterDirSS << std::setw(nchars) << std::setfill('0') << zIndex;
            fs::create_directory(outputDir / iterDirSS.str());

            // Dump chain, map, reslice for every particle
            for (size_t i = 0; i < nextVs.size(); ++i) {
                cv::Mat chain =
                    draw_particle_on_slice_(currentCurve, zIndex, i);
                cv::Mat resliceMat = reslices[i].draw();
                cv::Mat map = maps[i].draw();
                std::stringstream stream;
                stream << std::setw(nchars) << std::setfill('0') << zIndex
                       << "_" << std::setw(nchars) << std::setfill('0') << i;
                const fs::path base = zIdxDir / stream.str();
                cv::imwrite(base.string() + "_chain.png", chain);
                cv::imwrite(base.string() + "_reslice.png", resliceMat);
                cv::imwrite(base.string() + "_map.png", map);
            }
        }

        ////////////////////////////////////////////////////////////////////////////////
        // 5. Set up for next iteration
        currentVs = nextVs;
        points.push_back(nextVs);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // 6. Output final mesh
    return ExportAsPCD(points);
}

cv::Vec3d LocalResliceSegmentation::estimate_normal_at_index_(
    const FittedCurve& currentCurve, int index)
{
    const cv::Vec3d currentVoxel = currentCurve(index);
    auto stRadius = static_cast<int>(
        std::ceil(pkg_.getMaterialThickness() / pkg_.getVoxelSize()) / 2);
    const auto eigenPairs = pkg_.volume().interpolatedEigenPairsAt(
        currentVoxel(0), currentVoxel(1), currentVoxel(2), stRadius);
    const double exp0 = std::log10(eigenPairs[0].first);
    const double exp1 = std::log10(eigenPairs[1].first);
    if (std::abs(exp0 - exp1) > 2.0) {
        return eigenPairs[0].second;
    }
    const auto tan3d = D1At(currentCurve.points(), index, 3);
    return tan3d.cross(cv::Vec3d{0, 0, 1});
}

volcart::OrderedPointSet<cv::Vec3d> ExportAsPCD(
    const std::vector<std::vector<Voxel>>& points)
{
    int rows = points.size();
    int cols = points[0].size();
    std::vector<cv::Vec3d> tempRow;
    volcart::OrderedPointSet<cv::Vec3d> cloud(cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Voxel v = points[i][j];
            tempRow.emplace_back(v(0), v(1), v(2));
        }
        cloud.pushRow(tempRow);
        tempRow.clear();
    }
    return cloud;
}

cv::Mat LocalResliceSegmentation::draw_particle_on_slice_(
    const FittedCurve& curve,
    int sliceIndex,
    int particleIndex,
    bool showSpline) const
{
    auto pkgSlice = pkg_.volume().getSliceDataCopy(sliceIndex);
    pkgSlice.convertTo(
        pkgSlice, CV_8UC3, 1.0 / std::numeric_limits<uint8_t>::max());
    cv::cvtColor(pkgSlice, pkgSlice, cv::COLOR_GRAY2BGR);

    // Superimpose interpolated currentCurve on window
    if (showSpline) {
        const int n = 500;
        double sum = 0;
        int i = 0;
        std::vector<cv::Point> contour;
        while (i < n && sum <= 1.0) {
            contour.emplace_back(curve.eval(sum));
            sum += 1.0 / (n - 1);
        }
        cv::polylines(pkgSlice, contour, false, BGR_BLUE, 1, cv::LINE_AA);
    } else {
        // Draw circles on the pkgSlice window for each point
        for (size_t i = 0; i < curve.size(); ++i) {
            cv::Point real{int(curve(i)(0)), int(curve(i)(1))};
            cv::circle(pkgSlice, real, (showSpline ? 2 : 1), BGR_GREEN, -1);
        }
    }

    // Only highlight a point if particleIndex isn't default -1
    if (particleIndex != -1) {
        const Voxel particle = curve(particleIndex);
        cv::circle(
            pkgSlice, {int(particle(0)), int(particle(1))},
            (showSpline ? 2 : 1), BGR_RED, -1);
    }

    return pkgSlice;
}
