// Author: Julian Shilliger, contribution to Volume Cartographer as part of the 2023 "Vesuvius Challenge", MIT License

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
    return static_cast<size_t>((std::abs(endIndex_ - startIndex) + smoothness_interpolation_distance_ + smoothness_interpolation_window_) / stepSize_);
}

// print curve points with the help of this function
static std::ostream& operator<<(std::ostream& os, const Voxel& voxel) {
    os << "(" << voxel[0] << ", " << voxel[1] << ", " << voxel[2] << ")";
    return os;
}

// Estimating the 2D normal to the curve in the z plane
cv::Vec2f OpticalFlowSegmentationClass::estimate_2d_normal_at_index_(const FittedCurve& curve, int index) {
    int prevIndex = index - 1;
    int nextIndex = index + 1;

    cv::Vec2f prevPoint, currPoint, nextPoint;
    currPoint = cv::Vec2f(curve(index)[0], curve(index)[1]);

    if (prevIndex >= 0) {
        prevPoint = cv::Vec2f(curve(prevIndex)[0], curve(prevIndex)[1]);
    }
    if (nextIndex < curve.size()) {
        nextPoint = cv::Vec2f(curve(nextIndex)[0], curve(nextIndex)[1]);
    }

    cv::Vec2f tangent;
    if (prevIndex >= 0 && nextIndex < curve.size()) {
        tangent = nextPoint - prevPoint;
    } else if (prevIndex >= 0) {
        tangent = currPoint - prevPoint;
    } else if (nextIndex < curve.size()) {
        tangent = nextPoint - currPoint;
    } else {
        // The curve has only one point, so the normal cannot be estimated
        throw std::runtime_error("The curve has only one point, normal cannot be estimated.");
    }

    cv::Vec2f normal(-tangent[1], tangent[0]);

    return - normal / cv::norm(normal);
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

// Function to compute the moving average of a specific point in a vector of Voxel points
Voxel OpticalFlowSegmentationClass::compute_moving_average(const std::vector<Voxel>& points, int index, int window_size) {
    int count = 0;
    cv::Vec3f sum(0, 0, 0);

    for (int j = -window_size / 2; j <= window_size / 2; ++j) {
        int neighbor_index = index + j;

        if (neighbor_index >= 0 && neighbor_index < int(points.size())) {
            sum += cv::Vec3f(points[neighbor_index][0], points[neighbor_index][1], points[neighbor_index][2]);
            count++;
        }
    }

    return Voxel(sum[0] / count, sum[1] / count, sum[2] / count);
}

std::vector<std::vector<Voxel>> OpticalFlowSegmentationClass::interpolatePoints(std::vector<std::vector<Voxel>> points, int window_size, bool backwards) {
    // std::cout << "Interpolating points..." << std::endl;
    // Find starting index of points[0][2] in masterCloud_
    if (points.size() == 0) {
        return points;
    }
    int i=0;
    // Previous segmentation is totally contained in new segmentation, cannot be used for interpolation and is discarded. CASE FOREWARDS
    if (!backwards && (points[0][0][2] < masterCloud_.getRow(i)[masterCloud_.width()-1][2])){
        return points;
    }
    for (; i < masterCloud_.height(); i++) {
        auto masterRowI = masterCloud_.getRow(i);
        // Found corresponding index in master cloud
        if (points[0][0][2] == masterRowI[masterCloud_.width()-1][2]){
            break;
        }
    }
    // Previous segmentation is totally contained in new segmentation, cannot be used for interpolation and is discarded. CASE BACKWARDS
    if (i == masterCloud_.height()) {
        return points;
    }
    // std::cout << "Starting index of points[0][0][2] in masterCloud_: " << i << std::endl;
    // if (i == masterCloud_.height() || masterCloud_.getRow(i)[masterCloud_.width()-1][2] != points[0][0][2]) {
    //     std::cout << "Error: Could not find starting index of points[0][0][2] in masterCloud_." << std::endl;
    //     return points;
    // }

    // Interpolate points from 0 to window_size * masterCloud_.width()
    for (int u = 0; u < 2*window_size; u++) {
        int masterRowIndex = i + (backwards ? -u : u);
        int pointsIndex = u;
        float interpolate_point = ((float)u + 1) / (2.0 * (float)window_size);
        float interpolate_mastercloud = 1 - interpolate_point;
        // Check indexes in range
        if (!(masterRowIndex >= 0 && masterRowIndex < masterCloud_.height() && pointsIndex >= 0 && pointsIndex < points.size())) {
            std::cout << "Error: masterRowIndex: " << masterRowIndex << ", pointsIndex: " << pointsIndex << std::endl;
            std::cout << "masterCloud_.height(): " << masterCloud_.height() << ", points.size(): " << points.size() << std::endl;
            continue;
        }
        // std::cout << "interpolate_point: " << interpolate_point << ", interpolate_mastercloud: " << interpolate_mastercloud << std::endl;
        for (int j = 0; j < masterCloud_.width(); j++) {
                // std::cout << "masterRowIndex: " << masterCloud_.getRow(masterRowIndex)[j][2] << ", pointsIndex: " << points[pointsIndex][j][2] << std::endl;
                points[pointsIndex][j] = Voxel(interpolate_point * points[pointsIndex][j][0] + interpolate_mastercloud * masterCloud_.getRow(masterRowIndex)[j][0], interpolate_point * points[pointsIndex][j][1] + interpolate_mastercloud * masterCloud_.getRow(masterRowIndex)[j][1], interpolate_point * points[pointsIndex][j][2] + interpolate_mastercloud * masterCloud_.getRow(masterRowIndex)[j][2]);
                // points[pointsIndex][j] = interpolate_point * points[pointsIndex][j] + interpolate_mastercloud * masterCloud_.getRow(masterRowIndex)[j];
        }
        // evenly space interpolated curve
        FittedCurve evenlyInterpolationCurve(points[pointsIndex], std::round(points[pointsIndex][0][2]));
        points[pointsIndex] = evenlyInterpolationCurve.evenlySpacePoints();
    }
    return points;
}

// Multithreaded computation of splitted curve segment
std::vector<Voxel> OpticalFlowSegmentationClass::computeCurve(
    FittedCurve currentCurve,
    Chain& currentVs,
    int zIndex,
    bool backwards)
{
    bool visualize = false;
    // Extract 2D image slices at zIndex and zIndex+1
    // cv::Mat slice1 = vol_->getSliceDataCopy(zIndex);

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
    x_max = std::min(vol_->sliceWidth() - 1, x_max + margin);
    y_max = std::min(vol_->sliceHeight() - 1, y_max + margin);

    // Extract the region of interest
    cv::Rect roi(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
    cv::Mat roiSlice1 = vol_->getSliceDataRect(zIndex, roi);
    // Select the next slice depending if backwards
    cv::Mat roiSlice2 = vol_->getSliceDataRect(zIndex + (backwards ? -1 : 1), roi);

    // Convert to grayscale and normalize the slices
    cv::Mat gray1, gray2;
    cv::normalize(roiSlice1, gray1, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::normalize(roiSlice2, gray2, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::Mat integral_img;
    cv::integral(gray2, integral_img, CV_32S);

    // Compute dense optical flow using Farneback method
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(gray1, gray2, flow, 0.5, 3, 15, 3, 7, 1.2, 0);

    // Canny edge detection
    cv::Mat edges2;
    // Calculate the mean of the whole grayscale image using the integral image
    double total_intensity = static_cast<double>(integral_img.at<int>(integral_img.rows - 1, integral_img.cols - 1));
    double grayMean = total_intensity / (gray2.rows * gray2.cols);
    int lowThreshold = 1.0 * grayMean; // Set min_threshold as 0.66 * mean
    int highThreshold = 1.8 * grayMean; // Set max_threshold as 1.33 * mean
    int apertureSize = 3;
    // Set the threshold value
    int threshold_value = 50; // You can change this value according to your needs

    // Apply the threshold operation on both grayscale images
    cv::Mat gray1_, gray2_;
    cv::threshold(gray1, gray1_, lowThreshold, 0, cv::THRESH_TOZERO);
    cv::threshold(gray2, gray2_, lowThreshold, 0, cv::THRESH_TOZERO);
    cv::Canny(gray2, edges2, lowThreshold, highThreshold, apertureSize, true);
    // print the number of edges and total pixels
    int count_edges = cv::countNonZero(edges2);
    // std::cout << "Number of edges: " << count_edges << " of " << edges2.rows * edges2.cols << " pixels" << std::endl;

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges2, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // Create a blank image for edges_filtered
    cv::Mat edges2_filtered = cv::Mat::zeros(edges2.size(), CV_8UC1);
    // Filter contours by length
    int minLength = 100;  // Set your desired minimum length here
    std::vector<std::vector<cv::Point>> filtered_contours;
    for (const auto& contour : contours)
    {
        if (cv::arcLength(contour, false) >= minLength)
        {
            filtered_contours.push_back(contour);
        }
    }

    // // Approximate curves for the contours
    // double epsilonFactor = 0.001; // Set the approximation accuracy factor here
    // std::vector<std::vector<cv::Point>> approx_curves(filtered_contours.size());
    // for (size_t i = 0; i < filtered_contours.size(); ++i)
    // {
    //     double epsilon = epsilonFactor * cv::arcLength(filtered_contours[i], true);
    //     cv::approxPolyDP(filtered_contours[i], approx_curves[i], epsilon, true);
    // }

    // // Draw the approximated curves on the blank image
    // for (const auto& curve : approx_curves)
    // {
    //     cv::polylines(edges2_filtered, curve, -1, cv::Scalar(255), 1);
    // }

    // Display the result
    // cv::imshow("Approximated Curves", edges2_filtered);
    // cv::waitKey(0);

    // Draw filtered contours on edges_filtered with thickness set to 1
    cv::drawContours(edges2_filtered, filtered_contours, -1, cv::Scalar(255), 1);


    count_edges = cv::countNonZero(edges2_filtered);
    // std::cout << "Number of edges: " << count_edges << " of " << edges2_filtered.rows * edges2_filtered.cols << " pixels" << std::endl;

    int count_found_edges = 0;
    int count_wrong_edges = 0;

    // Initialize the updated curve
    std::vector<Voxel> nextVs;
    std::vector<Voxel> edgedVs;
    int black_treshold_imitate_brighter_pixel_movement = optical_flow_pixel_threshold_;
    int black_treshold_detect_outside = outside_threshold_;
    int window_size = 6; // Set the desired window size for averaging with an parameter? - should be fine for now. TODO: for adding different scroll resolution support
    int maxDistance = edge_jump_distance_; // Max distance to an considerable edge
    int whiteDistance = edge_bounce_distance_; // The distance the point is moved into the white part of the sheet
    std::vector<bool> updated_indices(currentCurve.size(), false);
    for (int i = 0; i < int(currentCurve.size()); ++i) {
        // Get the current point
        Voxel pt_ = currentCurve(i);
        // pt as a cv::Point2f
        cv::Point2f pt(pt_[0], pt_[1]);
        // Convert pt to ROI coordinates
        cv::Point2f roiPt = pt - cv::Point2f(x_min, y_min);
        // Estimate the normal at the current index
        // Handle edge cases with a window size of 5
        int lower_bound = std::max(0, i - 2);
        int upper_bound = std::min(i + 2, int(currentCurve.size()) - 1);

        cv::Vec2f mean_normal(0, 0);
        int window_size_normal = 0;

        for (int j = lower_bound; j <= upper_bound; ++j) {
            // Estimate the normal at index j
            cv::Vec2f normal = estimate_2d_normal_at_index_(currentCurve, j);
            mean_normal += normal;
            window_size_normal++;
        }

        // Calculate the mean normal for index i
        cv::Vec2f normal = mean_normal / window_size_normal;
        // {
        //     std::unique_lock<std::shared_mutex> lock(display_mutex_);
        //     std::cout << "Normal: " << normal << " Length" << cv::norm(normal) << std::endl;
        // }

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

        // Update ROI point
        roiPt = updatedPt - cv::Point2f(x_min, y_min);

        // Edge point update logic
        if (enable_edge_ && i != 0 && i != int(currentCurve.size()) - 1) {
            // Find the first edge along the normal up to maxDistance
            cv::Point2f edgePtFront, edgePtBack;
            bool foundFront = false, foundBack = false;
            bool disregardeFront = false, disregardeBack = false;
            for (int d = 0; d <= maxDistance+whiteDistance; ++d) {
                // {
                //     std::unique_lock<std::shared_mutex> lock(display_mutex_);
                //     std::cout << "Distance: " << d << cv::Point2f(d * normal[0], d * normal[1]) << std::endl;
                // }
                cv::Point2f candidatePtFront = roiPt + cv::Point2f(d * normal[0], d * normal[1]);
                cv::Point2f candidatePtBack = roiPt - cv::Point2f(d * normal[0], d * normal[1]);
                // std::cout << "Candidate point front: " << candidatePtFront << " Candidate point back: " << candidatePtBack << " Border: " << edges2.cols << " " << edges2.rows << std::endl;
                // this is the front side (towards center of scroll when initial points were placed counter clockwise)
                if (!disregardeFront && !foundFront && candidatePtFront.x >= 0 && candidatePtFront.x < edges2.cols &&
                    candidatePtFront.y >= 0 && candidatePtFront.y < edges2.rows && (edges2_filtered.at<uchar>(candidatePtFront) == 255 || edges2.at<uchar>(candidatePtFront) == 255)) {
                    count_found_edges++; // Count the number of found edges
                    auto gray2_val_back = get_mean_pixel_value(integral_img, candidatePtFront - cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]), window_size);
                    auto gray2_val_front = get_mean_pixel_value(integral_img, candidatePtFront + cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]), window_size);
                    if (gray2_val_back > gray2_val_front && gray2_val_back > black_treshold_detect_outside) {
                        edgePtFront = candidatePtFront + cv::Point2f(x_min, y_min) - cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]);
                        foundFront = true;
                        if ((edges2_filtered.at<uchar>(candidatePtFront) == 255)) {
                            disregardeFront = true;
                        }
                    }
                    else if (gray2_val_front > gray2_val_back && gray2_val_front > black_treshold_detect_outside && gray2_val_back < black_treshold_detect_outside) {
                        edgePtFront = candidatePtFront + cv::Point2f(x_min, y_min) + cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]);
                        foundFront = true;
                        if (!(edges2_filtered.at<uchar>(candidatePtFront) == 255)) {
                            disregardeFront = true;
                        }
                    }
                    else {
                        disregardeFront = true;
                        count_wrong_edges++;
                    }
                }
                // this is the backside
                if (!disregardeBack && !foundBack && candidatePtBack.x >= 0 && candidatePtBack.x < edges2.cols &&
                    candidatePtBack.y >= 0 && candidatePtBack.y < edges2.rows && (edges2_filtered.at<uchar>(candidatePtBack) == 255 || edges2.at<uchar>(candidatePtBack) == 255)) {
                    count_found_edges++; // Count the number of found edges
                    auto gray2_val_back = get_mean_pixel_value(integral_img, candidatePtBack + cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]), window_size);
                    auto gray2_val_front = get_mean_pixel_value(integral_img, candidatePtBack - cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]), window_size);
                    if (gray2_val_back > gray2_val_front && gray2_val_back > black_treshold_detect_outside && gray2_val_front < black_treshold_detect_outside) {
                        edgePtBack = candidatePtBack + cv::Point2f(x_min, y_min) + cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]);
                        foundBack = true;
                        if (!(edges2_filtered.at<uchar>(candidatePtBack) == 255)) {
                            disregardeBack = true;
                        }
                    }
                    else if (gray2_val_front > gray2_val_back && gray2_val_front > black_treshold_detect_outside) {
                        edgePtBack = candidatePtBack + cv::Point2f(x_min, y_min) - cv::Point2f(whiteDistance * normal[0], whiteDistance * normal[1]);
                        foundBack = true;
                        if (!(edges2_filtered.at<uchar>(candidatePtBack) == 255)) {
                            disregardeBack = true;
                        }
                    }
                    else {
                        disregardeBack = true;
                        count_wrong_edges++;
                    }
                }

                if ((foundFront || disregardeFront) && (foundBack || disregardeBack)) break;
            }
            // Update the point based on the found edges
            bool needs_update = false;
            if (foundFront && foundBack) {
                float frontDist = cv::norm(edgePtFront - updatedPt);
                float backDist = cv::norm(updatedPt - edgePtBack);
                if (0.45 * frontDist < backDist || frontDist - 5 < backDist) {
                    if (!disregardeFront) {
                        updatedPt = edgePtFront;
                        needs_update = true;
                    }
                } else {
                    if (!disregardeBack) {
                        updatedPt = edgePtBack;
                        needs_update = true;
                    }
                }
            } else if (foundFront && !disregardeFront) {
                updatedPt = edgePtFront;
                needs_update = true;
            } else if (foundBack && !disregardeBack) {
                updatedPt = edgePtBack;
                needs_update = true;
            }
            if (needs_update) {
                edgedVs.push_back(Voxel(updatedPt.x, updatedPt.y, zIndex + (backwards ? -1 : 1)));
                for (int u=0; u < 5; u++) {
                    if (i > u) updated_indices[i - u] = true;
                    if (i < int(currentCurve.size()) - u) updated_indices[i + u] = true;
                }
            }
            else {
                edgedVs.push_back(Voxel(0, 0, zIndex + (backwards ? -1 : 1)));
            }
        }

        // Add the updated point to the updated curve
        nextVs.push_back(Voxel(updatedPt.x, updatedPt.y, zIndex + (backwards ? -1 : 1)));
    }

    std::vector<Voxel> rawVs = nextVs;

    // Smooth black pixels by moving them closer to the edge
    // Smooth very bright pixels by moving them closer to the edge
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

            nextVs[i] = Voxel(projection[0], projection[1], zIndex + (backwards ? -1 : 1));
        }
    }

    // Parameters for filtering
    std::vector<Voxel> smoothedVs;
    std::vector<Voxel> interpolatedVs = nextVs;
    if (enable_smoothen_outlier_) {
        float distance_threshold = 10.0f;  // Adjust this threshold based on your specific requirements
        int interpolation_window = 3;  // Size of the window for interpolation

        // Interpolate the points if they are considered outliers
        for (int i = 1; i < int(nextVs.size()-1); ++i) {
            Voxel curr = nextVs[i];
            Voxel prev = nextVs[(i - 1 + nextVs.size()) % nextVs.size()];
            Voxel next = nextVs[(i + 1) % nextVs.size()];

            float left_distance = cv::norm(cv::Vec2f(curr[0] - prev[0], curr[1] - prev[1]));
            float right_distance = cv::norm(cv::Vec2f(curr[0] - next[0], curr[1] - next[1]));
            float mean_distance = (left_distance + right_distance) / 2.0f;

            if (mean_distance >= distance_threshold) {
                // Check if the point is an edge case
                bool is_edge_case = (i - interpolation_window / 2 < 0) || (i + interpolation_window / 2 >= int(nextVs.size()));

                if (is_edge_case) {
                    // Update the point along its normal
                    cv::Vec2f normal = estimate_2d_normal_at_index_(currentCurve, i);
                    Voxel averaged_point = compute_moving_average(nextVs, i, interpolation_window);
                    cv::Vec2f averaged_point_2d(averaged_point[0], averaged_point[1]);

                    float projectionLength = (averaged_point_2d - cv::Vec2f(curr[0], curr[1])).dot(normal);
                    cv::Vec2f projected_point = cv::Vec2f(curr[0], curr[1]) + projectionLength * normal;

                    interpolatedVs[i] = Voxel(projected_point[0], projected_point[1], curr[2]);
                } else {
                    // Compute the moving average of the current point
                    interpolatedVs[i] = compute_moving_average(nextVs, i, interpolation_window);
                }
            }
        }

        // Apply a moving average filter to smoothen the remaining updated points
        for (int i = 0; i < int(interpolatedVs.size()); ++i) {
            if (i == 0 || i == int(interpolatedVs.size()) - 1 || !updated_indices[i]) {
                smoothedVs.push_back(interpolatedVs[i]);
                continue;
            }
            else {
                smoothedVs.push_back(compute_moving_average(interpolatedVs, i, interpolation_window));
            }
        }
    }
    else {
        smoothedVs = nextVs;
    }

    // Display the edges_filtered image
    if (visualize) {
        // Apply a moving average filter to smoothen the remaining updated points
        std::vector<Voxel> displayVs0;
        for (int i = 0; i < int(nextVs.size()); ++i) {
            displayVs0.push_back(currentVs[i] - Voxel(x_min, y_min, -1));
        }
        FittedCurve newChain0(displayVs0, zIndex + (backwards ? -1 : 1));
        auto chain0 = draw_particle_on_image_(newChain0, edges2_filtered.clone());

        std::vector<Voxel> displayVsE;
        cv::Mat chainE = edges2_filtered.clone();
        chainE.convertTo(
            chainE, CV_8UC3, 1.0 / std::numeric_limits<uint8_t>::max());
        cv::cvtColor(chainE, chainE, cv::COLOR_GRAY2BGR);
        for (int i = 0; i < int(nextVs.size()); ++i) {
            if ((edgedVs[i] - Voxel(x_min, y_min, 0))[0] > 0) {
                cv::Point real{int((edgedVs[i] - Voxel(x_min, y_min, 0))(0)), int((edgedVs[i] - Voxel(x_min, y_min, 0))(1))};
                cv::circle(chainE, real, 2, BGR_GREEN, -1);
            }
        }
        chainE.convertTo(
            chainE, CV_8UC3, std::numeric_limits<uint8_t>::max());
        cv::cvtColor(chainE, chainE, cv::COLOR_BGR2GRAY);

        std::vector<Voxel> displayVs;
        for (int i = 0; i < int(nextVs.size()); ++i) {
            displayVs.push_back(rawVs[i] - Voxel(x_min, y_min, 0));
        }
        FittedCurve newChain(displayVs, zIndex + (backwards ? -1 : 1));
        auto chain = draw_particle_on_image_(newChain, edges2_filtered.clone());
        std::vector<Voxel> displayVs1;
        for (int i = 0; i < int(nextVs.size()); ++i) {
            displayVs1.push_back(nextVs[i] - Voxel(x_min, y_min, 0));
        }
        FittedCurve newChain1(displayVs1, zIndex + (backwards ? -1 : 1));
        auto chain1 = draw_particle_on_image_(newChain1, edges2_filtered.clone());
        std::vector<Voxel> displayVs2;
        for (int i = 0; i < int(interpolatedVs.size()); ++i) {
            displayVs2.push_back(interpolatedVs[i] - Voxel(x_min, y_min, 0));
        }
        FittedCurve newChain2(displayVs2, zIndex + (backwards ? -1 : 1));
        auto chain2 = draw_particle_on_image_(newChain2, edges2_filtered.clone());
        std::vector<Voxel> displayVs3;
        for (int i = 0; i < int(smoothedVs.size()); ++i) {
            displayVs3.push_back(smoothedVs[i] - Voxel(x_min, y_min, 0));
        }
        FittedCurve newChain3(displayVs3, zIndex + (backwards ? -1 : 1));
        auto chain3 = draw_particle_on_image_(newChain3, edges2_filtered.clone());

        std::unique_lock<std::shared_mutex> lock(display_mutex_);

        // stitch the three images together
        cv::Mat stitched;
        cv::hconcat(gray2, edges2, stitched);
        cv::hconcat(stitched, edges2_filtered, stitched);
        cv::hconcat(stitched, chain0, stitched);
        cv::hconcat(stitched, chainE, stitched);
        cv::hconcat(stitched, chain, stitched);
        cv::hconcat(stitched, chain1, stitched);
        cv::hconcat(stitched, chain2, stitched);
        cv::hconcat(stitched, chain3, stitched);
        cv::namedWindow("Next curve", cv::WINDOW_NORMAL);
        cv::imshow("Next curve", stitched);
        cv::waitKey(0);
    }
    // print the number of found edges
    // std::cout << "Found " << count_found_edges << " edges" << std::endl;
    // std::cout << "Found " << count_wrong_edges << " wrong edges" << std::endl;
    // print the edgedVs
    // std::cout << edgedVs << std::endl;
    // std::cout << nextVs << std::endl;

    // Return the updated vector of Voxel points
    return smoothedVs;
}

OpticalFlowSegmentationClass::PointSet OpticalFlowSegmentationClass::compute()
{
    // Max cache size
    if (nr_cache_slices_ >= 0 && vol_->getCacheCapacity() != nr_cache_slices_) {
        std::cout << "[Info]: Setting Cache Size to " << nr_cache_slices_ << " Slices" << std::endl;
        vol_->setCacheCapacity(nr_cache_slices_);
    }
    // Cache gets corrupted somewhere. One case: If estimate_normal_at_index_ from local reslice particle sim is used in multithreading mode.
    // Purge it to have clean state. Might take longer to process files.
    if (purge_cache_) {
        std::cout << "[Info]: Purging Slice Cache" << std::endl;
        vol_->cachePurge();
    }
    // Reset progress
    progressStarted();

    // Calculate the starting index
    auto minZPoint = std::min_element(
        begin(startingChain_), end(startingChain_),
        [](auto a, auto b) { return a[2] < b[2]; });
    auto startIndex = static_cast<int>(std::floor((*minZPoint)[2]));

    if (endIndex_ < 0) {
        throw std::domain_error("end index out of the volume");
    }
    bool backwards = startIndex > endIndex_;
    // Interpolation start = side/edge of the interpolation window that is nearest to start index
    // Example for backwards: | End Slice (50) | Interpolation End (70) | Interpolation Start (80) | Start Slice (100) |
    int interpolationStart = startIndex + (backwards ? -smoothness_interpolation_distance_ + smoothness_interpolation_window_  - 1: smoothness_interpolation_distance_ - smoothness_interpolation_window_ + 1);
    int interpolationEnd = startIndex + (backwards ? -smoothness_interpolation_distance_ - smoothness_interpolation_window_ : smoothness_interpolation_distance_ + smoothness_interpolation_window_);
    int interpolationLength = smoothness_interpolation_distance_ + smoothness_interpolation_window_;
    if (interpolationEnd < 0) {
        interpolationEnd = 0;
    }
    if (vol_->numSlices() <= interpolationEnd) {
        interpolationEnd = vol_->numSlices() - 1;
    }
    // std::cout << "Interpolation Window and Distance: " << smoothness_interpolation_window_ << " " << smoothness_interpolation_distance_ << std::endl;

    // Update the user-defined boundary
    bb_.setUpperBoundByIndex(2, (backwards ? startIndex : endIndex_) + 1);
    bb_.setLowerBoundByIndex(2, (backwards ? endIndex_ : startIndex) - 1);

    // Check that incoming points are all within bounds
    if (std::any_of(begin(startingChain_), end(startingChain_), [this](auto v) {
            return !bb_.isInBounds(v) || !vol_->isInBounds(v);
        })) {
        status_ = Status::ReturnedEarly;
        progressComplete();
        return create_final_pointset_({startingChain_});
    }
    if (std::any_of(begin(reSegStartingChain_), end(reSegStartingChain_), [this](auto v) {
            return !bb_.isInBounds(v) || !vol_->isInBounds(v);
        })) {
        status_ = Status::ReturnedEarly;
        progressComplete();
        return create_final_pointset_({startingChain_});
    }

    const fs::path outputDir("debugvis");
    const fs::path wholeChainDir(outputDir / "whole_chain");
    if (dumpVis_) {
        fs::create_directory(outputDir);
        fs::create_directory(wholeChainDir);
    }

    // Collection to hold all positions
    std::vector<std::vector<Voxel>> reSegPoints;
    std::vector<std::vector<Voxel>> points;

    if (smoothness_interpolation_window_ > 0) {
        reSegPoints.reserve((std::abs(endIndex_ - interpolationStart) + 1 + 1) / static_cast<uint64_t>(stepSize_));
    }
    points.reserve((std::abs(interpolationEnd - startIndex) + 1 + 1) / static_cast<uint64_t>(stepSize_));

    // Iterate over z-slices
    size_t iteration{0};

    if (smoothness_interpolation_window_ > 0 && !reSegStartingChain_.empty()) {

        // 1. If interpolation is active: Re-segment from the end index till start of interpolation window (overwrite existing points)
        if (computeSub(reSegPoints, reSegStartingChain_, endIndex_, interpolationStart, !backwards, iteration, !backwards, outputDir, wholeChainDir) == Status::ReturnedEarly) {
            return create_final_pointset_(reSegPoints);
        }

        // Overwrite points in local master cloud, so we can later use it for interpolation
        if (reSegPoints.size() > 0) {

            int i = 0;
            int pointIndex = 0;
            for (i = 0; i < masterCloud_.height(); i++) {
                auto masterRow = masterCloud_.getRow(i);
                if ((backwards ? endIndex_ + 1 : interpolationStart) == masterRow[0][2]){
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

            // Now that we have update the master cloud, we only want to retain the portion of points that is outside
            // the interpolation window.
            if (backwards) {
                reSegPoints.erase(reSegPoints.begin() + interpolationEnd - endIndex_ - 1, reSegPoints.end());
            } else {
                reSegPoints.erase(reSegPoints.begin() + interpolationStart - startIndex + 1, reSegPoints.end());
            }
        }

        // 2. If interpolation is active: Segment from start index till end of interpolation window (interpolate)
        if (computeSub(points, startingChain_, startIndex, interpolationEnd, backwards, iteration, true, outputDir, wholeChainDir) == Status::ReturnedEarly) {
            return create_final_pointset_(points);
        }

        points = interpolatePoints(points, smoothness_interpolation_window_, !backwards);

        // Merge the re-segmentation points with the "main" points
        if (!reSegPoints.empty()) {
            points.insert((backwards ? points.begin() : points.end()), reSegPoints.begin(), reSegPoints.end());
        }


    } else {
        // 3. If interpolation is not active: Segment from start index till end index
        if (computeSub(points, startingChain_, startIndex, endIndex_, backwards, iteration, backwards, outputDir, wholeChainDir) == Status::ReturnedEarly) {
            return create_final_pointset_(points);
        }
    }

    /////////////////////////////////////////////////////////
    // Update progress
    progressComplete();

    // 6. Output final mesh
    return create_final_pointset_(points);
}

ChainSegmentationAlgorithm::Status OpticalFlowSegmentationClass::computeSub(std::vector<std::vector<Voxel>>& points, Chain& currentVs, int startIndex, int endIndex, bool backwards,
    size_t& iteration, bool insertFront, const fs::path outputDir, const fs::path wholeChainDir)
{
    for (int zIndex = startIndex; backwards ? zIndex > endIndex : zIndex < endIndex;
         zIndex += backwards ? -stepSize_ : stepSize_) {

        // Update progress
        progressUpdated(iteration++);

        // Directory to dump vis
        std::stringstream ss;
        ss << std::setw(std::to_string(endIndex_).size()) << std::setfill('0')
           << zIndex;
        const fs::path zIdxDir = outputDir / ss.str();

        // std::cout << "Length curve points " << currentVs.size() << std::endl;

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
        const int max_points_per_thread = 25;
        int total_points = currentVs.size();
        int num_available_threads = static_cast<int>(std::thread::hardware_concurrency());
        int num_threads = std::max(1, std::min(static_cast<int>(std::floor(((float)total_points) / (float)min_points_per_thread)), num_available_threads - 1));
        int points_per_thread = std::min(max_points_per_thread, static_cast<int>(std::floor(((float)total_points) / (float)num_threads)));
        num_threads = static_cast<int>(std::floor(((float)total_points) / (float)points_per_thread));
        int num_concurrent_threads = std::min(num_threads, num_available_threads);
        int base_segment_length = static_cast<int>(std::floor(((float)total_points) / (float)num_threads));
        int num_threads_with_extra_point = total_points % num_threads;

        // std::cout << "Total points: " << total_points << " Num threads: " << num_threads << " Points per thread: " << points_per_thread << " Num concurrent threads: " << num_concurrent_threads << " Base segment length: " << base_segment_length << " Num threads with extra point: " << num_threads_with_extra_point;

        std::vector<std::vector<Voxel>> subsegment_points(num_threads);

        // Parallel computation of curve segments
        // Dispatch at most num_available_threads jobs at once. Repeat until all num_threads jobs are done.
        int start_idx = 0;
        for (int job = 0; job < num_threads; job += num_concurrent_threads)
        {
            int num_threads_to_dispatch = std::min(num_concurrent_threads, num_threads - job);
            std::vector<std::thread> threads(num_threads_to_dispatch);

            std::vector<std::vector<Voxel>> subsegment_vectors(num_threads_to_dispatch);
            for (int i = job; i < job+num_threads_to_dispatch; ++i)
            {
                int segment_length = base_segment_length + (i < num_threads_with_extra_point ? 1 : 0);
                int end_idx = start_idx + segment_length;
                // Change start_idx and end_idx to include overlap
                int start_idx_padded = (i == 0) ? 0 : (start_idx - 2);
                int end_idx_padded = (i == num_threads - 1) ? total_points : (end_idx + 2);
                std::vector<Voxel> subsegment(currentVs.begin() + start_idx_padded, currentVs.begin() + end_idx_padded);
                subsegment_vectors[i-job] = subsegment;
                start_idx = end_idx;
            }

            for (int i = job; i < job+num_threads_to_dispatch; ++i)
            {
                threads[i-job] = std::thread([&, i, job]()
                {
                    Chain subsegment_chain(subsegment_vectors[i-job]);
                    FittedCurve subsegmentCurve(subsegment_chain, zIndex);
                    std::vector<Voxel> subsegmentNextVs = computeCurve(subsegmentCurve, subsegment_chain, zIndex, backwards);
                    subsegment_points[i] = subsegmentNextVs;
                });
            }

            // Join threads
            for (auto& thread : threads)
            {
                thread.join();
            }
        }

        // Stitch curve segments together, discarding overlapping points
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
        FittedCurve stitchedFittedCurve(stitched_curve, zIndex + (backwards ? -1 : 1));
        std::vector<Voxel> nextVs = stitchedFittedCurve.evenlySpacePoints();

        // Check if any points in nextVs are outside volume boundaries. If so,
        // stop iterating and dump the resulting pointcloud.
        if (std::any_of(begin(nextVs), end(nextVs), [this](auto v) {
                return !bb_.isInBounds(v) || !vol_->isInBounds(v);
            })) {
            std::cout << "Returned early due to out-of-bounds points" << std::endl;
            status_ = Status::ReturnedEarly;
            return status_;
        }

        /////////////////////////////////////////////////////////
        // 4. Visualize if specified by user
        if (visualize_) {
            // Since points can change due to 2nd deriv optimization after main
            // optimization, refit a curve and draw that
            FittedCurve newChain(nextVs, zIndex + (backwards ? -1 : 1));
            auto chain = draw_particle_on_slice_(newChain, zIndex + (backwards ? -1 : 1));
            cv::namedWindow("Next curve", cv::WINDOW_NORMAL);
            cv::imshow("Next curve", chain);
            cv::waitKey(0);
        }

        /////////////////////////////////////////////////////////
        // 5. Set up for next iteration
        currentVs = nextVs;

        if (insertFront) {
            points.insert(points.begin(), nextVs);
        } else {
            points.push_back(nextVs);
        }
    }

    return status_;
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
    bool backwards = points[0][0](2) > points[rows - 1][cols - 1](2);
    std::vector<cv::Vec3d> tempRow;
    result_.clear();
    result_.setWidth(cols);

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            int index = backwards ? (rows - i - 1) : i;
            Voxel v = points[index][j];
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

cv::Mat OpticalFlowSegmentationClass::draw_particle_on_image_(
    const FittedCurve& curve,
    cv::Mat pkgSlice,
    int particleIndex,
    bool showSpline) const
{
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
    pkgSlice.convertTo(
        pkgSlice, CV_8UC3, std::numeric_limits<uint8_t>::max());
    cv::cvtColor(pkgSlice, pkgSlice, cv::COLOR_BGR2GRAY);
    return pkgSlice;
}
