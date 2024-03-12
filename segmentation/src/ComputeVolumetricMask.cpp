#include "vc/segmentation/ComputeVolumetricMask.hpp"

#include <map>
#include <opencv2/imgproc.hpp>

#include "vc/core/util/Iteration.hpp"
#include "vc/segmentation/tff/FloodFill.hpp"

using namespace volcart;
using namespace volcart::segmentation;

using Voxel = cv::Vec3i;
using VoxelList = std::vector<Voxel>;

void ComputeVolumetricMask::setLowThreshold(std::uint16_t t) { low_ = t; }

void ComputeVolumetricMask::setHighThreshold(std::uint16_t t) { high_ = t; }

void ComputeVolumetricMask::setEnableClosing(bool b) { enableClosing_ = b; }

void ComputeVolumetricMask::setClosingKernelSize(int s) { kernel_ = s; }

void ComputeVolumetricMask::setMeasureVertical(bool b)
{
    measureVertically_ = b;
}

void ComputeVolumetricMask::setMaxRadius(std::size_t radius)
{
    maxRadius_ = radius;
}

auto ComputeVolumetricMask::compute() -> VolumetricMask::Pointer
{
    // Setup the output
    mask_ = VolumetricMask::New();

    // Initialize running points with the provided starting seeds
    // Converts double-to-int by truncation
    auto startSlice = std::numeric_limits<std::size_t>::max();
    std::size_t endSlice{0};
    std::map<std::size_t, VoxelList> seedsBySlice;
    for (const auto& pt : input_) {
        auto sliceIdx = static_cast<std::size_t>(pt[2]);
        startSlice = std::min(startSlice, sliceIdx);
        endSlice = std::max(endSlice, sliceIdx);
        seedsBySlice[sliceIdx].emplace_back(pt[0], pt[1], pt[2]);
    }

    // Signal progress has begun
    progressStarted();

    // Iterate over z-slices
    for (auto zIndex : range(startSlice, endSlice + 1)) {
        // Update progress
        progressUpdated(zIndex - startSlice);

        // Get this slice's seed points
        VoxelList seedPoints;
        try {
            seedPoints = seedsBySlice.at(zIndex);
        } catch (const std::out_of_range& e) {
            // No seeds for this slice. Skip.
            continue;
        }

        // Get the current (single) slice image (Of type Mat)
        auto slice = vol_->getSliceDataCopy(zIndex);

        // Estimate thickness of page from every seed point.
        std::vector<std::size_t> estimates;
        for (const auto& v : seedPoints) {
            estimates.emplace_back(MeasureThickness(
                v, slice, low_, high_, measureVertically_, maxRadius_));
        }

        // Calculate the median thickness.
        // Choose the median of the measurements to be the boundary for every
        // point.
        auto bound = Median(estimates);

        // Do flood-fill with the given seed points to the estimated thickness.
        auto sliceMask = DoFloodFill(seedPoints, bound, slice, low_, high_);

        // Apply closing to fill holes and gaps.
        if (enableClosing_) {
            // Convert mask to a binary image so we can apply closing
            cv::Mat binaryImg = cv::Mat::zeros(slice.size(), CV_8UC1);
            for (const Voxel& v : sliceMask) {
                binaryImg.at<std::uint8_t>(v[1], v[0]) = 255;
            }

            cv::Mat kernel = cv::Mat::ones(kernel_, kernel_, CV_8U);
            cv::Mat closedImg;
            cv::morphologyEx(binaryImg, closedImg, cv::MORPH_CLOSE, kernel);

            // Save to the full volume mask
            for (const auto p : range2D(closedImg.rows, closedImg.cols)) {
                const auto& x = p.second;
                const auto& y = p.first;
                if (closedImg.at<std::uint8_t>(y, x) > 0) {
                    mask_->setIn({x, y, zIndex});
                }
            }
        } else {
            mask_->setIn(sliceMask);
        }
    }
    progressComplete();
    return mask_;
}

void ComputeVolumetricMask::setPointSet(const PointSet& ps) { input_ = ps; }

void ComputeVolumetricMask::setVolume(const Volume::Pointer& v) { vol_ = v; }

auto ComputeVolumetricMask::getMask() const -> VolumetricMask::Pointer
{
    return mask_;
}

auto ComputeVolumetricMask::progressIterations() const -> std::size_t
{
    if (input_.empty()) {
        return 0;
    }
    auto minmax = std::minmax_element(
        input_.begin(), input_.end(),
        [](const auto& a, const auto& b) { return a[2] < b[2]; });
    auto startSlice = static_cast<std::size_t>((*minmax.first)[2]);
    auto endSlice = static_cast<std::size_t>((*minmax.second)[2]);
    return endSlice + 1 - startSlice;
}
