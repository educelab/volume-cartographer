#pragma once

#include <vector>

#include <Eigen/SparseCore>
#include <opencv2/core.hpp>

#include "vc/core/types/NDArray.hpp"

namespace volcart
{

/**
 * @class VolumeMask
 * @brief Per-voxel Segmentation State Mask for Volumes
 *
 * Stores the segmentation state (segmented/unsegmented) for each voxel
 * of a Volume.
 *
 * @warning The underlying data structure for this class is memory efficient
 * but extremely slow. This class should be not used in production code.
 */
class VolumeMask
{
public:
    /** @brief Segmentation States */
    enum class State { Unsegmented = 0, Segmented };

    /** Subvolume containing voxel states */
    using Subvolume = NDArray<State>;

    /** @brief Construct from Volume dimensions */
    VolumeMask(size_t width, size_t height, size_t numSlices);

    /** @brief Set the segmentation state for a voxel */
    void setVoxelState(const cv::Vec3i& xyz, State state);

    /** @brief Get the segmentation state for a voxel */
    State getVoxelState(const cv::Vec3i& xyz);

    /**
     * @brief Set the segmentation state for every voxel in a subvolume
     *
     * @param origin Top-left corner of subvolume
     * @param dims Length of the subvolume's basis axes in voxels
     */
    void setSubvolumeState(
        const cv::Vec3i& origin, const cv::Vec3i& dims, State state);

    /**
     * @brief Get the segmentation state for every voxel in a subvolume
     *
     * @param origin Top-left corner of subvolume
     * @param dims Length of the subvolume's basis axes in voxels
     */
    Subvolume getSubvolumeState(const cv::Vec3i& origin, const cv::Vec3i& dims);

private:
    /**
     * Voxel state storage matrix
     *
     * The Eigen matrix is a 2D storage array. To accommodate a 3D volume,
     * z-indices are stored sequentially
     */
    Eigen::SparseMatrix<int, Eigen::RowMajor> states_;

    /** Slice width */
    size_t sliceWidth_;

    /** Slice height */
    size_t sliceHeight_;
};

}  // namespace volcart