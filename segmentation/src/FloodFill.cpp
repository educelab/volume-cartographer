#include "vc/segmentation/tff/FloodFill.hpp"

#include <queue>
#include <unordered_set>

using namespace volcart;
using namespace volcart::segmentation;

namespace vcs = volcart::segmentation;

using Voxel = cv::Vec3i;
using VoxelList = std::vector<cv::Vec3i>;
using VoxelSet = std::unordered_set<Voxel, Vec3iHash>;

struct VoxelPair {
    VoxelPair() = default;
    VoxelPair(const Voxel& v, const Voxel& parent) : v{v}, parent{parent} {}

    Voxel v;
    Voxel parent;
};

auto vcs::GetNeighbors(const cv::Vec3i& v) -> std::vector<cv::Vec3i>
{
    return {{v[0] - 1, v[1] - 1, v[2]}, {v[0], v[1] - 1, v[2]},
            {v[0] + 1, v[1] - 1, v[2]}, {v[0] - 1, v[1], v[2]},
            {v[0] + 1, v[1], v[2]},     {v[0] - 1, v[1] + 1, v[2]},
            {v[0], v[1] + 1, v[2]},     {v[0] + 1, v[1] + 1, v[2]}};
}

auto vcs::EuclideanDistance(const cv::Vec3i& start, const cv::Vec3i& end) -> int
{
    return static_cast<int>(cv::norm(end - start));
}

auto vcs::MeasureThickness(
    const cv::Vec3i& seed,
    const cv::Mat& slice,
    std::uint16_t low,
    std::uint16_t high,
    bool measureVert,
    std::size_t maxRadius) -> std::size_t
{
    int xPos{seed[0]};
    int xNeg{seed[0]};
    int yPos{seed[1]};
    int yNeg{seed[1]};
    bool foundMin{false};
    bool foundMax{false};
    std::size_t length{1};

    while (!foundMin || !foundMax) {
        if (measureVert) {
            yPos++;
            yNeg--;
        } else {
            xPos++;
            xNeg--;
        }

        // We've found our bound if we're out of the image bounds now
        foundMin = xNeg < 0 or yNeg < 0;
        foundMax = xPos >= slice.cols or yPos >= slice.rows;

        // Check the negative direction
        if (!foundMin) {
            auto val = slice.at<std::uint16_t>(yNeg, xNeg);
            if (val < low or val > high) {
                foundMin = true;
            }
        }

        // Check the positive direction
        if (!foundMax) {
            auto val = slice.at<std::uint16_t>(yPos, xPos);
            if (val < low or val > high) {
                foundMax = true;
            }
        }

        if (!(foundMin && foundMax)) {
            length++;
        }

        // Break if our length is at the the max
        if (length == maxRadius) {
            break;
        }
    }
    return length;
}

auto vcs::DoFloodFill(
    const VoxelList& pts,
    int bound,
    cv::Mat img,
    std::uint16_t low,
    std::uint16_t high) -> VoxelList
{
    std::queue<VoxelPair> q;
    VoxelList mask;
    VoxelSet visited;

    // Push all the initial points onto the queue.
    // Initial points are their own 'parents'.
    for (const auto& pt : pts) {
        auto greyVal = img.at<std::uint16_t>(pt[1], pt[0]);
        if (greyVal >= low && greyVal <= high) {
            q.emplace(pt, pt);
            visited.insert(pt);
        }
    }

    while (!q.empty()) {
        // Pick a VoxelPair off the queue
        auto pair = q.front();
        q.pop();

        //'color'/record that cv::Vec3i as part of the mask
        mask.push_back(pair.v);

        // check neighbors; if they're valid according to the user-defined
        // threshold AND they are not outside the original(/parent) seed point's
        // boundary, add them to the queue
        for (const auto& neighbor : GetNeighbors(pair.v)) {
            // Make sure this cv::Vec3i hasn't already been added to the
            // visited list: (We don't want to add it to the queue
            // twice...)
            if (visited.find(neighbor) != visited.end()) {
                continue;
            }

            // Make sure this neighbor is in the image bounds
            if (neighbor[0] < 0 or neighbor[0] >= img.cols or neighbor[1] < 0 or
                neighbor[1] >= img.rows) {
                continue;
            }

            // Add the valid neighbor to the queue and mark it as visited.
            auto val = img.at<std::uint16_t>(neighbor[1], neighbor[0]);
            auto dist = EuclideanDistance(neighbor, pair.parent);
            if (val >= low && val <= high && dist <= bound) {
                q.emplace(neighbor, pair.parent);
                visited.insert(neighbor);
            }
        }
    }
    return mask;
}