#include <deque>
#include <iomanip>
#include <limits>
#include <list>
#include <tuple>
#include <algorithm>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/math/StructureTensor.hpp"
#include "vc/segmentation/OpticalFlowSegmentation.hpp"
#include "vc/segmentation/lrps/Common.hpp"
#include "vc/segmentation/lrps/Derivative.hpp"
#include "vc/segmentation/lrps/EnergyMetrics.hpp"
#include "vc/segmentation/lrps/FittedCurve.hpp"
#include "vc/segmentation/lrps/IntensityMap.hpp"

using namespace volcart::segmentation;
namespace fs = volcart::filesystem;
using std::begin;
using std::end;

size_t OpticalFlowSegmentationClass::progressIterations() const
{
    auto minZPoint = std::min_element(
        startingChain_.begin(), startingChain_.end(),
        [](auto a, auto b) { return a[2] < b[2]; });
    auto startIndex = static_cast<int>(std::floor((*minZPoint)[2]));
    return static_cast<size_t>((endIndex_ - startIndex) / stepSize_);
}

// print curve points with the help of this function
static std::ostream& operator<<(std::ostream& os, const Voxel& voxel) {
    os << "(" << voxel[0] << ", " << voxel[1] << ", " << voxel[2] << ")";
    return os;
}

// estimting the 2D normal to the curve in the z plane
cv::Vec2f OpticalFlowSegmentationClass::estimate_2d_normal_at_index_(const FittedCurve& curve, int index) {
    int prevIndex = (index - 1 + curve.size()) % curve.size();
    int nextIndex = (index + 1) % curve.size();

    cv::Vec2f prevPoint(curve(prevIndex)[0], curve(prevIndex)[1]);
    cv::Vec2f currPoint(curve(index)[0], curve(index)[1]);
    cv::Vec2f nextPoint(curve(nextIndex)[0], curve(nextIndex)[1]);

    cv::Vec2f tangent = nextPoint - prevPoint;
    cv::Vec2f normal(-tangent[1], tangent[0]);

    return normal / cv::norm(normal);
}

// fast method to get mean pixel value of window_size by using an integral image
float OpticalFlowSegmentationClass::get_mean_pixel_value(const cv::Mat& integral_img, const cv::Point& pt, int window_size) {
    int x_min = std::max(pt.x - window_size / 2, 0);
    int x_max = std::min(pt.x + window_size / 2, integral_img.cols - 2);
    int y_min = std::max(pt.y - window_size / 2, 0);
    int y_max = std::min(pt.y + window_size / 2, integral_img.rows - 2);

    int a = integral_img.at<int>(y_min, x_min);
    int b = integral_img.at<int>(y_min, x_max + 1);
    int c = integral_img.at<int>(y_max + 1, x_min);
    int d = integral_img.at<int>(y_max + 1, x_max + 1);

    float sum = static_cast<float>(a + d - b - c);
    int count = (x_max - x_min + 1) * (y_max - y_min + 1);

    return sum / count;
}

// Multithreaded computation of splitted curve segment
std::vector<Voxel> OpticalFlowSegmentationClass::computeCurve(
    FittedCurve currentCurve,
    Chain& currentVs,
    int zIndex) 
{
    // Extract 2D image slices at zIndex and zIndex+1
    cv::Mat slice1 = vol_->getSliceDataCopy(zIndex);
    cv::Mat slice2 = vol_->getSliceDataCopy(zIndex+1);

    // Calculate the bounding box of the curve to define the region of interest
    int x_min = std::numeric_limits<int>::max();
    int y_min = std::numeric_limits<int>::max();
    int x_max = std::numeric_limits<int>::min();
    int y_max = std::numeric_limits<int>::min();

    for (int i = 0; i < int(currentCurve.size()); ++i) {
        // Get the current point
        Voxel pt_ = currentCurve(i);
        x_min = std::min(x_min, static_cast<int>(pt_[0]));
        y_min = std::min(y_min, static_cast<int>(pt_[1]));
        x_max = std::max(x_max, static_cast<int>(pt_[0]));
        y_max = std::max(y_max, static_cast<int>(pt_[1]));
    }

    // Add a margin to the bounding box to avoid edge effects
    int margin = 15;
    x_min = std::max(0, x_min - margin);
    y_min = std::max(0, y_min - margin);
    x_max = std::min(slice1.cols - 1, x_max + margin);
    y_max = std::min(slice1.rows - 1, y_max + margin);

    // Extract the region of interest
    cv::Rect roi(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
    cv::Mat roiSlice1 = slice1(roi);
    cv::Mat roiSlice2 = slice2(roi);

    // Convert to grayscale and normalize the slices
    cv::Mat gray1, gray2;
    cv::normalize(roiSlice1, gray1, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::normalize(roiSlice2, gray2, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat integral_img;
    cv::integral(gray2, integral_img, CV_32S);


    // Compute dense optical flow using Farneback method
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(gray1, gray2, flow, 0.5, 3, 15, 3, 7, 1.2, 0);

    // Initialize the updated curve
    std::vector<Voxel> nextVs;
    int black_treshold_imitate_brighter_pixel_movement = optical_flow_pixel_threshold_;
    for (int i = 0; i < int(currentCurve.size()); ++i) {
        // Get the current point
        Voxel pt_ = currentCurve(i);
        // pt as a cv::Point2f
        cv::Point2f pt(pt_[0], pt_[1]);
        // Convert pt to ROI coordinates
        cv::Point2f roiPt = pt - cv::Point2f(x_min, y_min);


        // Get the optical flow vector at the current point
        cv::Vec2f flowVec = flow.at<cv::Vec2f>(roiPt);

        // Check if the flow magnitude is more than optical_flow_displacement_threshold_ pixels
        if (cv::norm(flowVec) > optical_flow_displacement_threshold_) {
            // Calculate the average flow around a 5x5 window
            int windowSize = 5;
            cv::Vec2f avgFlow(0, 0);
            int count = 0;
            for (int x = -windowSize / 2; x <= windowSize / 2; ++x) {
                for (int y = -windowSize / 2; y <= windowSize / 2; ++y) {
                    cv::Point2f neighborPt = roiPt + cv::Point2f(x, y);
                    if (neighborPt.x >= 0 && neighborPt.x < flow.cols && neighborPt.y >= 0 && neighborPt.y < flow.rows) {
                        int neighborIntensity = gray2.at<uchar>(neighborPt);
                        if (neighborIntensity > black_treshold_imitate_brighter_pixel_movement) {
                            avgFlow += flow.at<cv::Vec2f>(neighborPt);
                            count++;
                        }
                    }
                }
            }
            if (count > 0) {
                flowVec = avgFlow / count;
            }
        }

        // Move the point along with respect to the optical flow vector
        cv::Point2f updatedPt = cv::Vec2f(pt_[0], pt_[1]) + flowVec;

        // Add the updated point to the updated curve
        nextVs.push_back(Voxel(updatedPt.x, updatedPt.y, zIndex + 1));
    }

    // Smooth black pixels by moving them closer to the edge
    // Smooth very bright pixels by moving them closer to the edge
    int black_treshold_detect_outside = outside_threshold_;
    int window_size = 6; // Set the desired window size for averaging with an parameter? - should be fine for now. TODO: for adding different scroll resolution support
    for (int i = 0; i < int(nextVs.size()); ++i) {
        Voxel curr = nextVs[i];
        int currIntensity = gray2.at<uchar>(cv::Point(curr[0] - x_min, curr[1] - y_min));
        cv::Point pt(curr[0] - x_min, curr[1] - y_min);
        float mean_intensity = get_mean_pixel_value(integral_img, pt, window_size);

        if (mean_intensity < black_treshold_detect_outside || currIntensity < black_treshold_detect_outside || currIntensity > smoothen_by_brightness_) {
            // Estimate the normal at the current index
            cv::Vec2f normal = estimate_2d_normal_at_index_(currentCurve, i);

            // Get the previous and next points
            Voxel prev = nextVs[(i - 1 + nextVs.size()) % nextVs.size()];
            Voxel next = nextVs[(i + 1) % nextVs.size()];

            // Calculate the direction vector between prev and next points
            cv::Vec2f direction(next[0] - prev[0], next[1] - prev[1]);
            direction /= cv::norm(direction);

            // Project the current point onto the line between prev and next points
            cv::Vec2f prevToCurr(curr[0] - prev[0], curr[1] - prev[1]);
            float projectionLength = prevToCurr.dot(direction);
            cv::Vec2f projection(prev[0] + projectionLength * direction[0], prev[1] + projectionLength * direction[1]);

            nextVs[i] = Voxel(projection[0], projection[1], zIndex + 1);
        }
    }

    // Return the updated vector of Voxel points
    return nextVs;
}

OpticalFlowSegmentationClass::PointSet OpticalFlowSegmentationClass::compute()
{
    // Max cache size
    if (nr_cache_slices_ >= 0 && vol_->getCacheCapacity() != nr_cache_slices_) {
        std::cout << "[Info]: Setting Cache Size to" << nr_cache_slices_ << " Slices" << std::endl;
        vol_->setCacheCapacity(nr_cache_slices_);
    }
    // Cache gets corrupted somewhere(one case: if estimate_normal_at_index_ from local reslice particle sim is used in multithreading mode). purge it to have clean state. might take longer to process files.
    // if purge_cache_ is true, purge cache
    if (purge_cache_) {
        std::cout << "[Info]: Purging Slice Cache" << std::endl;
        vol_->cachePurge();
    }
    // Reset progress
    progressStarted();

    // Duplicate the starting chain
    auto currentVs = startingChain_;

    // Update the user-defined boundary
    bb_.setUpperBoundByIndex(2, endIndex_ + 1);

    // Check that incoming points are all within bounds
    if (std::any_of(begin(currentVs), end(currentVs), [this](auto v) {
            return !bb_.isInBounds(v) || !vol_->isInBounds(v);
        })) {
        status_ = Status::ReturnedEarly;
        progressComplete();
        return create_final_pointset_({currentVs});
    }

    const fs::path outputDir("debugvis");
    const fs::path wholeChainDir(outputDir / "whole_chain");
    if (dumpVis_) {
        fs::create_directory(outputDir);
        fs::create_directory(wholeChainDir);
    }

    // Calculate the starting index
    auto minZPoint = std::min_element(
        begin(currentVs), end(currentVs),
        [](auto a, auto b) { return a[2] < b[2]; });
    auto startIndex = static_cast<int>(std::floor((*minZPoint)[2]));

    if (endIndex_ <= startIndex) {
        throw std::domain_error("end index <= start index");
    }

    // Collection to hold all positions
    std::vector<std::vector<Voxel>> points;
    points.reserve(
        (endIndex_ - startIndex + 1) / static_cast<uint64_t>(stepSize_));
    points.push_back(currentVs);

    // Iterate over z-slices
    size_t iteration{0};
    for (int zIndex = startIndex; zIndex <= endIndex_; zIndex += stepSize_) {
        // Update progress
        progressUpdated(iteration++);

        // Directory to dump vis
        std::stringstream ss;
        ss << std::setw(std::to_string(endIndex_).size()) << std::setfill('0')
           << zIndex;
        const fs::path zIdxDir = outputDir / ss.str();

        //////////////////////////////////////////////////////////
        // 0. Resample current positions so they are evenly spaced
        FittedCurve currentCurve(currentVs, zIndex);
        currentVs = currentCurve.evenlySpacePoints();

        // Dump entire curve for easy viewing
        if (dumpVis_) {
            ss.str(std::string());
            ss << std::setw(std::to_string(endIndex_).size())
                << std::setfill('0') << zIndex << "_chain.png";
            const auto wholeChainPath = wholeChainDir / ss.str();
            cv::imwrite(
                wholeChainPath.string(),
                draw_particle_on_slice_(currentCurve, zIndex, -1, true));
        }

        // Split the curve into subsegments
        // Calculate num_threads and segment_length
        const int min_points_per_thread = 15;
        int total_points = currentVs.size();
        int num_threads = std::max(1, std::min(static_cast<int>(std::floor(((float)total_points) / (float)min_points_per_thread)), static_cast<int>(std::thread::hardware_concurrency()) - 1));
        int base_segment_length = static_cast<int>(std::floor(((float)total_points) / (float)num_threads));
        int num_threads_with_extra_point = total_points % num_threads;
        
        std::vector<std::vector<Voxel>> subsegment_points(num_threads);

        // Parallel computation of curve segments
        std::vector<std::thread> threads(num_threads);
        std::vector<std::vector<Voxel>> subsegment_vectors(num_threads);
        int start_idx = 0;
        for (int i = 0; i < num_threads; ++i)
        {
            int segment_length = base_segment_length + (i < num_threads_with_extra_point ? 1 : 0);
            int end_idx = start_idx + segment_length;
            // Change start_idx and end_idx to include overlap
            int start_idx_padded = (i == 0) ? 0 : (start_idx - 2);
            int end_idx_padded = (i == num_threads - 1) ? total_points : (end_idx + 2);
            std::vector<Voxel> subsegment(currentVs.begin() + start_idx_padded, currentVs.begin() + end_idx_padded);
            subsegment_vectors[i] = subsegment;
            start_idx = end_idx;
        }

        for (int i = 0; i < num_threads; ++i)
        {

            threads[i] = std::thread([&, i]()
            {
                Chain subsegment_chain(subsegment_vectors[i]);
                FittedCurve subsegmentCurve(subsegment_chain, zIndex);
                std::vector<Voxel> subsegmentNextVs = computeCurve(subsegmentCurve, subsegment_chain, zIndex);
                subsegment_points[i] = subsegmentNextVs;
            });
        }

        // Join threads and stitch curve segments together
        for (auto& thread : threads)
        {
            thread.join();
        }
        std::vector<Voxel> stitched_curve;
        stitched_curve.reserve(currentVs.size());

        // Stitch curve segments together, discarding overlapping points
        for (int i = 0; i < num_threads; ++i)
        {
            if (i > 0) {
                subsegment_points[i].erase(subsegment_points[i].begin(), subsegment_points[i].begin() + 2);
            }
            if (i < num_threads - 1) {
                subsegment_points[i].erase(subsegment_points[i].end() - 2, subsegment_points[i].end());
            }
            stitched_curve.insert(stitched_curve.end(), subsegment_points[i].begin(), subsegment_points[i].end());
        }
        
        // Generate nextVs by evenly spacing points in the stitched curve
        FittedCurve stitchedFittedCurve(stitched_curve, zIndex);
        std::vector<Voxel> nextVs = stitchedFittedCurve.evenlySpacePoints();

        // Check if any points in nextVs are outside volume boundaries. If so,
        // stop iterating and dump the resulting pointcloud.
        if (std::any_of(begin(nextVs), end(nextVs), [this](auto v) {
                return !bb_.isInBounds(v) || !vol_->isInBounds(v);
            })) {
            std::cout << "Returned early due to out-of-bounds points" << std::endl;
            status_ = Status::ReturnedEarly;
            return create_final_pointset_(points);
        }

        /////////////////////////////////////////////////////////
        // 4. Visualize if specified by user
        if (visualize_) {
            // Since points can change due to 2nd deriv optimization after main
            // optimization, refit a curve and draw that
            FittedCurve newChain(nextVs, zIndex + 1);
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

cv::Vec3d OpticalFlowSegmentationClass::estimate_normal_at_index_(
    const FittedCurve& currentCurve, int index)
{
    auto currentVoxel = currentCurve(index);
    auto radius = static_cast<int>(
        std::ceil(materialThickness_ / vol_->voxelSize()) * 0.5);
    auto eigenPairs = ComputeSubvoxelEigenPairs(vol_, currentVoxel, radius);
    double exp0 = std::log10(eigenPairs[0].first);
    double exp1 = std::log10(eigenPairs[1].first);
    cv::Vec3d normal;
    if (std::abs(exp0 - exp1) > 2.0) {
        normal = eigenPairs[0].second;
    } else {
        auto tan3d = D1At(currentCurve.points(), index, 3);
        normal = tan3d.cross(cv::Vec3d{0, 0, 1});
    }

    // Normalize the normal vector
    double norm = cv::norm(normal);
    if (norm > 0) {
        normal /= norm;
    }

    return normal;
}

OpticalFlowSegmentationClass::PointSet
OpticalFlowSegmentationClass::create_final_pointset_(
    const std::vector<std::vector<Voxel>>& points)
{
    auto rows = points.size();
    auto cols = points[0].size();
    std::vector<cv::Vec3d> tempRow;
    result_.clear();
    result_.setWidth(cols);

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            Voxel v = points[i][j];
            tempRow.emplace_back(v(0), v(1), v(2));
        }
        result_.pushRow(tempRow);
        tempRow.clear();
    }
    return result_;
}

cv::Mat OpticalFlowSegmentationClass::draw_particle_on_slice_(
    const FittedCurve& curve,
    int sliceIndex,
    int particleIndex,
    bool showSpline) const
{
    auto pkgSlice = vol_->getSliceDataCopy(sliceIndex);
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
            cv::circle(pkgSlice, real, 2, BGR_GREEN, -1);
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