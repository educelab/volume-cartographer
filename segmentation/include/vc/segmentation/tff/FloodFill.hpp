#pragma once

/** @file */

#include <vector>

#include <opencv2/core.hpp>

#include "vc/core/util/HashFunctions.hpp"

namespace volcart::segmentation
{

/** Get the list of a voxel's eight neighbors */
std::vector<cv::Vec3i> GetNeighbors(const cv::Vec3i& v);

/** Calculate the Euclidean distance between two voxels */
int EuclideanDistance(const cv::Vec3i& start, const cv::Vec3i& end);

/**
 * Estimate the thickness of page from every seed point.
 *
 * Projects bidirectionally from the seed until it finds voxels not in the range
 * `[low, high]` or until `maxRadius` is reached. If `measureVert` is true,
 * projects vertically from seed. Otherwise, projects horizontally.
 */
size_t MeasureThickness(
    const cv::Vec3i& seed,
    const cv::Mat& slice,
    uint16_t low,
    uint16_t high,
    bool measureVert,
    size_t maxRadius);

/** Find the median element in a container */
template <class Container>
auto Median(Container c)
{
    std::nth_element(c.begin(), c.begin() + c.size() / 2, c.end());
    return c[c.size() / 2];
}

/**
 * Run flood fill using the provided set of seed points
 *
 * Returns the contiguous set of points which fall within the range [low, high]
 * and which are no more than `bound` distance from an initial seed.
 */
std::vector<cv::Vec3i> DoFloodFill(
    const std::vector<cv::Vec3i>& pts,
    int bound,
    cv::Mat img,
    uint16_t low,
    uint16_t high);

}  // namespace volcart::segmentation