#include "vc/segmentation/OpticalFlowSegmentation.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <optional>
#include <thread>
#include <tuple>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/ImageIO.hpp"
#include "vc/core/math/StructureTensor.hpp"
#include "vc/core/types/Color.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/String.hpp"
#include "vc/segmentation/lrps/Derivative.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"

// #include <sys/stat.h>

using namespace volcart::segmentation;
namespace fs = volcart::filesystem;

using PointSet = OpticalFlowSegmentation::PointSet;
using RawPointSet = std::vector<std::vector<Voxel>>;
using Chain = OpticalFlowSegmentation::Chain;
using OpticalFlowSegmentation::Status;

// estimating the 2D normal to the curve in the z plane
namespace
{
template <class Iterable, typename RetType = int>
auto minimum_elem(const Iterable& iterable, std::size_t elem = 2) -> RetType
{
    auto it = std::min_element(
        std::begin(iterable), std::end(iterable),
        [elem](const auto& a, const auto& b) { return a[elem] < b[elem]; });
    return static_cast<RetType>((*it)[elem]);
}

auto Estimate2DNormalAtIndex(const FittedCurve& curve, const std::size_t index)
    -> cv::Vec2f
{
    const auto prevIndex = (index - 1 + curve.size()) % curve.size();
    const auto nextIndex = (index + 1) % curve.size();

    const auto p = curve(static_cast<int>(prevIndex));
    const auto i = curve(static_cast<int>(index));
    const auto n = curve(static_cast<int>(nextIndex));

    const cv::Vec2f prevPt(static_cast<float>(p[0]), static_cast<float>(p[1]));
    const cv::Vec2f currPt(static_cast<float>(i[0]), static_cast<float>(i[1]));
    const cv::Vec2f nextPt(static_cast<float>(n[0]), static_cast<float>(n[1]));

    const auto tangent = nextPt - prevPt;
    const cv::Vec2f normal(-tangent[1], tangent[0]);

    return normal / cv::norm(normal);
}

// fast method to get mean pixel value of window size by using an integral image
auto GetMeanPixelValue(
    const cv::Mat& integralImg,
    const cv::Point& pt,
    const int windowSize) -> float
{
    const int xMin = std::max(pt.x - windowSize / 2, 0);
    const int xMax = std::min(pt.x + windowSize / 2, integralImg.cols - 2);
    const int yMin = std::max(pt.y - windowSize / 2, 0);
    const int yMax = std::min(pt.y + windowSize / 2, integralImg.rows - 2);

    const auto a = integralImg.at<std::int32_t>(yMin, xMin);
    const auto b = integralImg.at<std::int32_t>(yMin, xMax + 1);
    const auto c = integralImg.at<std::int32_t>(yMax + 1, xMin);
    const auto d = integralImg.at<std::int32_t>(yMax + 1, xMax + 1);

    const auto sum = static_cast<float>(a + d - b - c);
    const auto cnt = static_cast<float>((xMax - xMin + 1) * (yMax - yMin + 1));

    return sum / cnt;
}

auto IsInBounds(const cv::Point2f& p, const cv::Mat& img)
{
    return p.x >= 0 and p.x < img.cols and p.y >= 0 and p.y < img.rows;
}

auto computeSub(
    Chain currentVs,
    int startChainIndex,
    int endChainIndex,
    int endIndex,
    int initialStepAdjustment,
    bool backwards,
    std::size_t& iteration,
    bool insertFront,
    const fs::path outputDir,
    const fs::path wholeChainDir) -> std::tuple<RawPointSet, Status>
{
    RawPointSet points;
    return {points, Status::Success};
}

auto Interpolate(
    bool backwards,
    int interpStart,
    int interpEnd,
    int interpWindow,
    int startIndex,
    int endIndex,
    double stepSize,
    const Chain& startingChain,
    int startIndexChain,
    const Chain& reSegStartingChain_,
    int startIndexReSegChain) -> RawPointSet
{
    // 1. If interpolation is active: Re-segment from the end index till start
    // of interpolation window (overwrite existing points)
    int adjustedInterpolBorder = interpStart;
    int initialStepAdjustment = 0;
    if (stepSize > 1) {
        // In case we are stepping, we might not exactly hit our interpolation
        // targets, so we might need to overshoot a bit to have enough curves
        // for the interpolation window.

        if (std::abs(startIndexReSegChain - startIndexChain) % (int)stepSize !=
            0) {
            // For the re-segmentation portion, we might have to use an offset
            // in case our range is not a multiple of the step size. If we would
            // not do that, our re-segmentation portion would access other
            // slices than the forward segmentation portion thus negating
            // partially the performance benefit of the step size (which has the
            // goal of using as few slices as possible).
            initialStepAdjustment =
                (backwards ? -1. : 1.) *
                (stepSize - (std::abs(startIndexReSegChain - startIndexChain) %
                             (int)stepSize));

            // Interpolation range is not a multiple of our step size => find
            // nearest usable one. If however, our current interpolation start
            // equals already the start slice (so around 100% interpolation)
            // then of course we cannot adjust, as there is no room to grow.
            if (interpStart != startIndex) {
                adjustedInterpolBorder =
                    interpStart -
                    (backwards ? -1 : 1) *
                        (stepSize -
                         (std::abs(startIndexReSegChain - interpStart) %
                          (int)stepSize)) +
                    initialStepAdjustment;
            }
        }
    }

    auto [reSegPoints, status] = computeSub(
        reSegStartingChain_, startIndexReSegChain, startIndexChain,
        adjustedInterpolBorder, initialStepAdjustment, !backwards, iteration,
        !backwards, outputDir, wholeChainDir);
    if (status == Status::ReturnedEarly) {
        return {startingChain};
    }
    if (status == Status::Failure) {
        // TODO: FAILURE
    }

    // For step sizes greater than 1 we have to interpolate the results
    if (stepSize > 1) {
        // Add re-segmentation chain to points for the gap closing and remove
        // again afterwards
        reSegPoints.insert(
            backwards ? reSegPoints.begin() : reSegPoints.end(),
            reSegStartingChain_);

        if (std::abs(interpStart - startIndexChain) < stepSize) {
            // Distance between interpolation start and the starting chain index
            // is less than step size
            // => we need to add the starting slice for the gap interpolation.
            reSegPoints.insert(
                backwards ? reSegPoints.end() : reSegPoints.begin(),
                startingChain);
        }

        reSegPoints = interpolateGaps(reSegPoints);
        // Remove re-segmentation chain
        reSegPoints.erase(backwards ? reSegPoints.begin() : reSegPoints.end());

        // Remove everything before the interpolation start index and also the
        // end anchor slice (which for very big interpolation percentages (~
        // 100%) might "accidentally" be included which is correct for the gap
        // interpolation, but needs to go now).
        reSegPoints.erase(
            std::remove_if(
                reSegPoints.begin(), reSegPoints.end(),
                [backwards, interpStart, startIndexReSegChain](auto row) {
                    return (backwards ? row[0][2] > interpStart
                                      : row[0][2] < interpStart) ||
                           row[0][2] == startIndexReSegChain;
                }),
            std::end(reSegPoints));
    }

    // Overwrite points in local master cloud, so we can later use it for
    // interpolation
    if (reSegPoints.size() > 0) {

        int i = 0;
        int pointIndex = 0;
        for (i = 0; i < masterCloud_.height(); i++) {
            auto masterRow = masterCloud_.getRow(i);
            if ((backwards ? endIndex : interpStart) == masterRow[0][2]) {
                pointIndex = i * masterCloud_.width();
                break;
            }
        }

        for (auto row : reSegPoints) {
            for (int j = 0; j < row.size(); j++) {
                masterCloud_[pointIndex] = row.at(j);
                pointIndex++;
            }
        }

        // Now that we have updated the master cloud, we only want to retain the
        // portion of resegmentation points that is outside the interpolation
        // window.
        reSegPoints.erase(
            std::remove_if(
                reSegPoints.begin(), reSegPoints.end(),
                [backwards, interpStart, interpEnd](auto row) {
                    return (
                        backwards
                            ? row[0][2] <= interpStart && row[0][2] >= interpEnd
                            : row[0][2] >= interpStart &&
                                  row[0][2] <= interpEnd);
                }),
            std::end(reSegPoints));
    }

    // 2. If interpolation is active: Segment from start index till end of
    // interpolation window (interpolate with existing points)
    adjustedInterpolBorder = interpEnd;
    if (stepSize > 1 && interpEnd != endIndex) {
        // In case we are stepping, we might not exactly hit our interpolation
        // targets, so we might need to overshoot a bit to have enough curves
        // for the interpolation window.
        if (std::abs(interpEnd - startIndexChain) % (int)stepSize != 0) {
            // Interpolation range is not a multiple of our step size => find
            // nearest usable one
            adjustedInterpolBorder =
                interpEnd +
                (backwards ? -1 : 1) *
                    (stepSize -
                     (std::abs(interpEnd - startIndexChain) % (int)stepSize));
        }
    }

    if (computeSub(
            points, startingChain, startIndexChain, startIndexReSegChain,
            adjustedInterpolBorder, 0, backwards, iteration, backwards,
            outputDir, wholeChainDir) == Status::ReturnedEarly) {
        return create_final_pointset_(points);
    }

    // For step sizes greater than 1 we have to interpolate the results
    if (stepSize > 1) {
        // Add starting chain to points for the gap closing and remove again
        // afterwards
        points.insert(
            (backwards ? points.end() : points.begin()), startingChain);

        if (std::abs(interpEnd - startIndexReSegChain) < stepSize) {
            // Distance between interpolation end and the re-segmentation
            // starting chain index is less than step size
            // => we need to add the starting slice for the gap interpolation.
            points.insert(
                backwards ? points.begin() : points.end(), reSegStartingChain_);
        }

        points = interpolateGaps(points);
        // Remove starting chain
        points.erase((backwards ? points.end() : points.begin()));

        // Remove everything beyond the interpolation end index and also the
        // start anchor slice (which for very big interpolation percentages (~
        // 100%) might "accidentally" be included which is correct for the gap
        // interpolation, but needs to go now).
        points.erase(
            std::remove_if(
                points.begin(), points.end(),
                [backwards, interpEnd, startIndexChain](auto row) {
                    return (backwards ? row[0][2] < interpEnd
                                      : row[0][2] > interpEnd) ||
                           row[0][2] == startIndexChain;
                }),
            std::end(points));
    }

    // Split the points into the overwrite portion and the interpolation
    // portion.
    auto interpolationPoints = RawPointSet(
        (backwards ? points.begin()
                   : points.begin() + interpStart - startIndex),
        (backwards ? points.begin() + interpStart - interpEnd + 1
                   : points.end()));
    points.erase(
        backwards ? points.begin() : points.begin() + interpStart - startIndex,
        backwards ? points.begin() + interpStart - interpEnd + 1
                  : points.end());

    interpolationPoints = interpolateWithMasterCloud(
        interpolationPoints, smoothness_interpolation_window_, !backwards);

    // Merge the interpolation and re-segmentation points with the "main" points
    // created from the actual starting slice curve
    points.insert(
        (backwards ? points.begin() : points.end()),
        interpolationPoints.begin(), interpolationPoints.end());
    points.insert(
        (backwards ? points.begin() : points.end()), reSegPoints.begin(),
        reSegPoints.end());
}
}  // namespace

void OpticalFlowSegmentation::setStartZIndex(const int z) { startIndex_ = z; }

auto OpticalFlowSegmentation::getStartZIndex() const -> int
{
    return startIndex_;
}

void OpticalFlowSegmentation::setTargetZIndex(const int z) { endIndex_ = z; }

auto OpticalFlowSegmentation::getTargetZIndex() const -> int
{
    return endIndex_;
}

void OpticalFlowSegmentation::setOptimizationIterations(const std::size_t n)
{
    numIters_ = n;
}

void OpticalFlowSegmentation::setOutsideThreshold(const std::uint8_t outside)
{
    outsideThreshold_ = outside;
}

void OpticalFlowSegmentation::setOFThreshold(const std::uint8_t ofThr)
{
    opticalFlowPixelThreshold_ = ofThr;
}

void OpticalFlowSegmentation::setOFDispThreshold(const std::uint32_t ofDispThrs)
{
    opticalFlowDisplacementThreshold_ = ofDispThrs;
}

void OpticalFlowSegmentation::setSmoothBrightnessThreshold(
    const std::uint8_t brightness)
{
    smoothByBrightness_ = brightness;
}

void OpticalFlowSegmentation::setEnableSmoothOutliers(const bool enable)
{
    enable_smoothen_outlier_ = enable;
}

void OpticalFlowSegmentation::setEnableEdgeDetection(const bool enable)
{
    enable_edge_ = enable;
}

void OpticalFlowSegmentation::setEdgeJumpDistance(const std::uint32_t distance)
{
    edge_jump_distance_ = distance;
}

void OpticalFlowSegmentation::setEdgeBounceDistance(
    const std::uint32_t distance)
{
    edge_bounce_distance_ = distance;
}

void OpticalFlowSegmentation::setInterpolate(bool b) { requestInterp_ = b; }

void OpticalFlowSegmentation::setInterpolationWindow(const std::uint32_t window)
{
    interpWindow_ = window;
}
auto OpticalFlowSegmentation::getInterpolationWindow() const -> std::uint32_t
{
    return interpWindow_;
}
void OpticalFlowSegmentation::setInterpolationDistance(
    const std::uint32_t distance)
{
    interpDist_ = distance;
}
auto OpticalFlowSegmentation::getInterpolationDistance() const -> std::uint32_t
{
    return interpDist_;
}
void OpticalFlowSegmentation::setMasterCloud(PointSet masterCloud)
{
    masterCloud_ = std::move(masterCloud);
}

void OpticalFlowSegmentation::setReSegmentationChain(Chain c)
{
    resegStartingChain_ = std::move(c);
}
void OpticalFlowSegmentation::setMaterialThickness(const double m)
{
    materialThickness_ = m;
}

void OpticalFlowSegmentation::setMaxThreads(std::uint32_t t)
{
    maxThreads_ = t;
}

void OpticalFlowSegmentation::resetMaxThreads() { maxThreads_.reset(); }

void OpticalFlowSegmentation::setVisualize(const bool b) { visualize_ = b; }

void OpticalFlowSegmentation::setDumpVis(const bool b) { dumpVis_ = b; }

auto OpticalFlowSegmentation::progressIterations() const -> std::size_t
{
    const auto minZPoint = std::min_element(
        startingChain_.begin(), startingChain_.end(),
        [](const auto& a, const auto& b) { return a[2] < b[2]; });
    const auto startIndex = static_cast<int>(std::floor((*minZPoint)[2]));
    return static_cast<std::size_t>((endIndex_ - startIndex) / stepSize_);
}

// Multithreaded computation of split curve segment
auto OpticalFlowSegmentation::compute_curve_(
    const FittedCurve& currentCurve, int zIndex) const -> std::vector<Voxel>
{
    // Extract 2D image slices at zIndex and zIndex+1
    const auto slice1 = vol_->getSliceDataCopy(zIndex);
    const auto slice2 = vol_->getSliceDataCopy(zIndex + 1);

    // Calculate the bounding box of the curve to define the region of interest
    int xMin = std::numeric_limits<int>::max();
    int yMin = std::numeric_limits<int>::max();
    int xMax = std::numeric_limits<int>::min();
    int yMax = std::numeric_limits<int>::min();
    for (int i = 0; i < currentCurve.size(); ++i) {
        auto point = currentCurve(i);
        xMin = std::min(xMin, static_cast<int>(point[0]));
        yMin = std::min(yMin, static_cast<int>(point[1]));
        xMax = std::max(xMax, static_cast<int>(point[0]));
        yMax = std::max(yMax, static_cast<int>(point[1]));
    }

    // Add a margin to the bounding box to avoid edge effects
    const int margin = 15;
    xMin = std::max(0, xMin - margin);
    yMin = std::max(0, yMin - margin);
    xMax = std::min(slice1.cols - 1, xMax + margin);
    yMax = std::min(slice1.rows - 1, yMax + margin);

    // Extract the region of interest
    const cv::Rect roi(xMin, yMin, xMax - xMin + 1, yMax - yMin + 1);
    const cv::Mat roiSlice1 = slice1(roi);
    const cv::Mat roiSlice2 = slice2(roi);

    // Convert to grayscale and normalize the slices
    cv::Mat gray1;
    cv::Mat gray2;
    cv::normalize(roiSlice1, gray1, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::normalize(roiSlice2, gray2, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat integralImg;
    cv::integral(gray2, integralImg, CV_32S);

    // Compute dense optical flow using Farneback method
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(gray1, gray2, flow, 0.5, 3, 15, 3, 7, 1.2, 0);

    // Calculate the average flow around a 5x5 window
    int windowSize = 5;
    const cv::Point2f minPt(static_cast<float>(xMin), static_cast<float>(yMin));
    std::vector<Voxel> nextVs;
    for (int i = 0; i < currentCurve.size(); ++i) {
        // Get the current point
        auto cp = currentCurve(i);
        const cv::Point2f pt(cp[0], cp[1]);

        // Convert pt to ROI coordinates
        const auto roiPt = pt - minPt;

        // Get the optical flow vector at the current point
        auto flowVec = flow.at<cv::Vec2f>(roiPt);

        // Check if the flow magnitude is more than opticalFlowDisplacementThreshold_ pixels
        if (cv::norm(flowVec) > opticalFlowDisplacementThreshold_) {
            cv::Vec2f avgFlow(0, 0);
            int count = 0;
            const auto bXY = -windowSize / 2;
            const auto eXY = 1 + windowSize / 2;
            for (const auto [x, y] : range2D(bXY, eXY, bXY, eXY)) {
                const cv::Point2f xyPt{
                    static_cast<float>(x), static_cast<float>(y)};
                const auto neighborPt = roiPt + xyPt;
                if (::IsInBounds(neighborPt, flow)) {
                    auto neighborIntensity = gray2.at<std::uint8_t>(neighborPt);
                    if (neighborIntensity > opticalFlowPixelThreshold_) {
                        avgFlow += flow.at<cv::Vec2f>(neighborPt);
                        count++;
                    }
                }
            }
            // Update the flow vec with the mean vec
            if (count > 0) {
                flowVec = avgFlow / count;
            }
        }

        // Move the point along with respect to the optical flow vector
        auto updatedPt = pt + cv::Point2f(flowVec);

        // Add the updated point to the updated curve
        nextVs.emplace_back(updatedPt.x, updatedPt.y, zIndex + 1);
    }

    // Smooth black pixels by moving them closer to the edge
    // Smooth very bright pixels by moving them closer to the edge
    windowSize = static_cast<int>(
        std::ceil(materialThickness_ / vol_->voxelSize()) * 0.25);
    for (int i = 0; i < nextVs.size(); ++i) {
        auto curr = nextVs[i];
        const cv::Point pt(
            static_cast<int>(curr[0]) - xMin, static_cast<int>(curr[1]) - yMin);
        auto currIntensity = gray2.at<std::uint8_t>(pt);
        auto meanIntensity = ::GetMeanPixelValue(integralImg, pt, windowSize);

        if (meanIntensity < static_cast<float>(outsideThreshold_) ||
            currIntensity < outsideThreshold_ ||
            currIntensity > smoothByBrightness_) {

            // Estimate the normal at the current index
            const auto normal = ::Estimate2DNormalAtIndex(currentCurve, i);

            // Get the previous and next points
            auto prev = nextVs[(i - 1 + nextVs.size()) % nextVs.size()];
            auto next = nextVs[(i + 1) % nextVs.size()];

            // Calculate the direction vector between prev and next points
            cv::Vec2d direction(next[0] - prev[0], next[1] - prev[1]);
            direction /= cv::norm(direction);

            // Project the current point onto the line between prev and next points
            const cv::Vec2d prevToCurr(curr[0] - prev[0], curr[1] - prev[1]);
            auto projLen = prevToCurr.dot(direction);
            const cv::Vec2d proj(
                prev[0] + projLen * direction[0],
                prev[1] + projLen * direction[1]);

            nextVs[i] = Voxel(proj[0], proj[1], zIndex + 1);
        }
    }

    // Return the updated vector of Voxel points
    return nextVs;
}

auto OpticalFlowSegmentation::compute() -> PointSet
{
    // Early check that the target makes sense
    if (endIndex_ < 0) {
        throw std::domain_error("End index out of the volume");
    }

    // Reset progress
    progressStarted();

    // Calculate the starting index
    auto startIndexChain = minimum_elem(startingChain_);
    int startIndexResegChain{-1};
    if (not resegStartingChain_.empty()) {
        startIndexResegChain = minimum_elem(resegStartingChain_);
    }

    // check if we're segment +/-Z
    bool backwards = startIndexChain > endIndex_;

    // Interpolation start = side/edge of the interpolation window that is
    // nearest to start index Example for backwards (forward is
    // mirrored/reveresed): | End Slice (50) | Interpolation End (70) |
    // Interpolation Start (80) | Start Slice (100) |
    auto dist = static_cast<std::int32_t>(interpDist_);
    auto win = static_cast<std::int32_t>(interpWindow_);
    int interpStart{startIndex_};
    int interpEnd{startIndex_};
    if (backwards) {
        interpStart -= dist + win;
        interpEnd -= dist - win;
    } else {
        interpStart += dist - win;
        interpEnd += dist + win;
    }
    interpStart = std::clamp(interpStart, 0, vol_->numSlices() - 1);
    interpEnd = std::clamp(interpEnd, 0, vol_->numSlices() - 1);
    if (interpStart == interpEnd) {
        // TODO: Error?
    }

    // Update the user-defined boundary
    if (backwards) {
        bb_.setUpperBoundByIndex(2, startIndexChain + 1);
        bb_.setLowerBoundByIndex(
            2, std::min(startIndexResegChain, endIndex_) - 1);
    } else {
        bb_.setUpperBoundByIndex(
            2, std::max(startIndexResegChain, endIndex_) + 1);
        bb_.setLowerBoundByIndex(2, startIndexChain - 1);
    }

    // Check that incoming points are all within bounds
    const auto inBounds = [&](const cv::Vec3d& v) -> bool {
        return not bb_.isInBounds(v) || not vol_->isInBounds(v);
    };
    if (std::any_of(startingChain_.begin(), startingChain_.end(), inBounds)) {
        status_ = Status::ReturnedEarly;
        progressComplete();
        Logger()->error("[OFS] Starting chain out of bounds");
        return create_final_pointset_({startingChain_});
    }
    if (std::any_of(
            resegStartingChain_.begin(), resegStartingChain_.end(), inBounds)) {
        status_ = Status::ReturnedEarly;
        progressComplete();
        Logger()->error("[OFS] Re-segmentation starting chain out of bounds");
        return create_final_pointset_({startingChain_});
    }

    // Create debug directories
    const fs::path outputDir("debugvis");
    const auto wholeChainDir = outputDir / "whole_chain";
    if (dumpVis_) {
        fs::create_directory(outputDir);
        fs::create_directory(wholeChainDir);
    }

    // Are we doing interp?
    const auto haveInterpD = interpDist_ > 0;
    const auto haveResegChain = not resegStartingChain_.empty();
    const auto haveMasterCloud = not masterCloud_.empty();
    const auto doInterp =
        requestInterp_ and haveInterpD and haveResegChain and haveMasterCloud;

    // Sanity checks
    if (requestInterp_ and not haveInterpD) {
        throw std::runtime_error(
            "Requested interpolation but interpolation distance is 0");
    }
    if (requestInterp_ and not haveResegChain) {
        throw std::runtime_error(
            "Requested interpolation but did not provide resegmentation chain");
    }
    if (requestInterp_ and not haveMasterCloud) {
        throw std::runtime_error(
            "Requested interpolation but did not provide master cloud");
    }

    // Do interpolation
    if (doInterp) {
    }

    // Iterate over z-slices
    std::size_t iteration{0};
    auto stepSize = static_cast<int>(stepSize_);
    const int padding = vol_->numSlices();
    for (int zIndex = startIndex; zIndex < endIndex_; zIndex += stepSize) {
        // Update progress
        progressUpdated(iteration++);

        // Directory to dump vis
        auto zStr = to_padded_string(zIndex, padding);
        const fs::path zIdxDir = outputDir / zStr;

        //////////////////////////////////////////////////////////
        // 0. Resample current positions so they are evenly spaced
        FittedCurve currentCurve(currentVs, zIndex);
        currentVs = currentCurve.evenlySpacePoints();

        // Dump entire curve for easy viewing
        if (dumpVis_) {
            const auto wholeChainPath = wholeChainDir / (zStr + "_chain.png");
            WriteImage(
                wholeChainPath,
                draw_particle_on_slice_(currentCurve, zIndex, -1, true));
        }

        // Set up the maximum number of threads
        auto maxThreads = std::thread::hardware_concurrency() - 1;
        if (maxThreads_.has_value()) {
            maxThreads = std::min(maxThreads, maxThreads_.value());
        }

        // Calculate the num threads we're going to use
        const std::uint32_t minPointsPerThread{15};
        const auto numPts = currentVs.size();
        const auto ptsPerThread = static_cast<std::uint32_t>(std::floor(
            static_cast<float>(numPts) /
            static_cast<float>(minPointsPerThread)));
        const auto numThreads =
            std::max(1U, std::min(ptsPerThread, maxThreads));
        const auto baseSegmentLength = static_cast<std::uint32_t>(std::floor(
            static_cast<float>(numPts) / static_cast<float>(numThreads)));
        const auto numThreadsWithExtraPoint = numPts % numThreads;

        // Parallel computation of curve segments
        RawPointSet subsegmentVectors;
        std::size_t startIdx{0};
        for (const auto& i : range(numThreads)) {
            auto segmentLength =
                baseSegmentLength + (i < numThreadsWithExtraPoint ? 1 : 0);
            auto endIdx = startIdx + segmentLength;

            // Change start_idx and end_idx to include overlap
            auto startIdxPadded =
                static_cast<std::int64_t>((i == 0) ? 0 : (startIdx - 2));
            auto endIdxPadded = static_cast<std::int64_t>(
                (i == numThreads - 1) ? numPts : (endIdx + 2));

            // Copy from currentVs to our vector
            auto startIt = std::next(currentVs.begin(), startIdxPadded);
            auto endIt = std::next(currentVs.begin(), endIdxPadded);
            subsegmentVectors.emplace_back(startIt, endIt);
            startIdx = endIdx;
        }

        // Initialize the threads
        std::vector<std::thread> threads;
        RawPointSet subsegmentPoints(numThreads);
        for (const auto& i : range(numThreads)) {
            threads.emplace_back(
                [this, &subsegmentVectors, &zIndex, &subsegmentPoints, i]() {
                    const Chain subsegmentChain(subsegmentVectors[i]);
                    const FittedCurve curve(subsegmentChain, zIndex);
                    subsegmentPoints[i] = compute_curve_(curve, zIndex);
                });
        }

        // Join threads and stitch curve segments together
        for (auto& thread : threads) {
            thread.join();
        }

        // Stitch curve segments together, discarding overlapping points
        std::vector<Voxel> stitched;
        stitched.reserve(currentVs.size());
        for (auto [i, segment] : enumerate(subsegmentPoints)) {
            auto startIt = segment.begin();
            auto endIt = segment.end();
            if (i > 0) {
                startIt = std::next(segment.begin(), 2);
            }
            if (i < numThreads - 1) {
                endIt = std::next(segment.end(), -2);
            }
            stitched.insert(stitched.end(), startIt, endIt);
        }

        // Generate nextVs by evenly spacing points in the stitched curve
        FittedCurve stitchedFittedCurve(stitched, zIndex + 1);
        auto nextVs = stitchedFittedCurve.evenlySpacePoints();

        // Check if any points in nextVs are outside volume boundaries. If so,
        // stop iterating and dump the resulting point cloud.
        if (std::any_of(begin(nextVs), end(nextVs), [this](const auto& v) {
                return !bb_.isInBounds(v) || !vol_->isInBounds(v);
            })) {
            status_ = Status::ReturnedEarly;
            return create_final_pointset_(points);
        }

        /////////////////////////////////////////////////////////
        // 4. Visualize if specified by user
        if (visualize_) {
            // Since points can change due to 2nd deriv optimization after main
            // optimization, refit a curve and draw that
            const FittedCurve newChain(nextVs, zIndex + 1);
            auto chain = draw_particle_on_slice_(newChain, zIndex + 1);
            cv::namedWindow("Next curve", cv::WINDOW_NORMAL);
            cv::imshow("Next curve", chain);
            cv::waitKey(0);
        }

        /////////////////////////////////////////////////////////
        // 5. Set up for next iteration
        currentVs = nextVs;
        points.push_back(nextVs);
    }

    /////////////////////////////////////////////////////////
    // Update progress
    progressComplete();

    // 6. Output final mesh
    return create_final_pointset_(points);
}

auto OpticalFlowSegmentation::create_final_pointset_(
    const std::vector<std::vector<Voxel>>& points) -> PointSet
{
    const auto rows = points.size();
    const auto cols = points[0].size();
    std::vector<cv::Vec3d> tempRow;
    result_.clear();
    result_.setWidth(cols);

    for (std::size_t i = 0; i < rows; ++i) {
        for (std::size_t j = 0; j < cols; ++j) {
            Voxel v = points[i][j];
            tempRow.emplace_back(v(0), v(1), v(2));
        }
        result_.pushRow(tempRow);
        tempRow.clear();
    }
    return result_;
}

auto OpticalFlowSegmentation::draw_particle_on_slice_(
    const FittedCurve& curve,
    const int sliceIndex,
    const int particleIndex,
    const bool showSpline) const -> cv::Mat
{
    auto pkgSlice = vol_->getSliceDataCopy(sliceIndex);
    pkgSlice.convertTo(
        pkgSlice, CV_8UC3, 1.0 / std::numeric_limits<std::uint8_t>::max());
    cv::cvtColor(pkgSlice, pkgSlice, cv::COLOR_GRAY2BGR);

    // Superimpose interpolated currentCurve on window
    if (showSpline) {
        const int n = 500;
        double sum = 0;
        const int i = 0;
        std::vector<cv::Point> contour;
        while (i < n && sum <= 1.0) {
            contour.emplace_back(curve.eval(sum));
            sum += 1.0 / (n - 1);
        }
        cv::polylines(pkgSlice, contour, false, color::BLUE, 1, cv::LINE_AA);
    } else {
        // Draw circles on the pkgSlice window for each point
        for (std::size_t i = 0; i < curve.size(); ++i) {
            const cv::Point real{int(curve(i)(0)), int(curve(i)(1))};
            cv::circle(pkgSlice, real, 2, color::GREEN, -1);
        }
    }

    // Only highlight a point if particleIndex isn't default -1
    if (particleIndex != -1) {
        const Voxel particle = curve(particleIndex);
        cv::circle(
            pkgSlice, {int(particle(0)), int(particle(1))},
            (showSpline ? 2 : 1), color::RED, -1);
    }

    return pkgSlice;
}
