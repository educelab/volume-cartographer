/**
 * @file
 *
 * @ingroup Math
 */

#pragma once

#include <opencv2/core.hpp>

#include "vc/core/math/Tensor3D.hpp"
#include "vc/core/types/Volume.hpp"

namespace volcart
{
/** Eigenvalue type */
using EigenValue = double;

/** Eigenvector type */
using EigenVector = cv::Vec3d;

/** EigenPair type */
using EigenPairs = std::array<std::pair<EigenValue, EigenVector>, 3>;

/** StructureTensor type */
using StructureTensor = cv::Matx33d;

/** Zero-valued StructureTensor */
static const auto ZERO_STRUCTURE_TENSOR =
    StructureTensor(0, 0, 0, 0, 0, 0, 0, 0, 0);

/**
 * @brief Compute the structure tensor for a voxel position
 *
 * The structure tensor is calculated from the gradient of a cubic subvolume
 * centered around the provided point. The size of this subvolume is defined
 * by `radius`.
 *
 * The `kernelSize` must be one of the following: 1, 3, 5, 7.
 * If `kernelSize = 3`, the Scharr operator will be used to
 * calculate the gradient, otherwise the Sobel operator will be used.
 *
 * More information about the structure tensor can be found on
 * <a href="https://en.wikipedia.org/wiki/Structure_tensor"> Wikipedia</a>.
 *
 * @param radius Radius of subvolume used to calculate structure tensor
 * @param kernelSize Size of the gradient kernel
 */
StructureTensor ComputeVoxelStructureTensor(
    const Volume::Pointer& volume,
    int vx,
    int vy,
    int vz,
    int radius = 1,
    int kernelSize = 3);

/** @overload ComputeVoxelStructureTensor() */
StructureTensor ComputeVoxelStructureTensor(
    const Volume::Pointer& volume,
    const cv::Vec3i& index,
    int radius = 1,
    int kernelSize = 3);

/**
 * @brief Compute the structure tensor for a subvoxel position
 *
 * @copydetails ComputeVoxelStructureTensor()
 *
 * This version computes the structure tensor for subvoxel positions within
 * the volume.
 */
StructureTensor ComputeSubvoxelStructureTensor(
    const Volume::Pointer& volume,
    double vx,
    double vy,
    double vz,
    int radius = 1,
    int kernelSize = 3);

/** @overload ComputeSubvoxelStructureTensor() */
StructureTensor ComputeSubvoxelStructureTensor(
    const Volume::Pointer& volume,
    const cv::Vec3d& index,
    int radius = 1,
    int kernelSize = 3);

/**
 * @brief Compute the eigenvalues and eigenvectors from the structure tensor
 * for a voxel position
 *
 * @copydetails ComputeVoxelStructureTensor()
 */
EigenPairs ComputeVoxelEigenPairs(
    const Volume::Pointer& volume,
    int x,
    int y,
    int z,
    int radius = 1,
    int kernelSize = 3);

/** @overload ComputeVoxelEigenPairs() */
EigenPairs ComputeVoxelEigenPairs(
    const Volume::Pointer& volume,
    const cv::Vec3i& index,
    int radius = 1,
    int kernelSize = 3);

/**
 * @brief Compute the eigenvalues and eigenvectors from the structure tensor
 * for a subvoxel position
 *
 * @copydetails ComputeVoxelEigenPairs()
 *
 * This version computes the structure tensor for subvoxel positions within
 * the volume.
 */
EigenPairs ComputeSubvoxelEigenPairs(
    const Volume::Pointer& volume,
    double x,
    double y,
    double z,
    int radius = 1,
    int kernelSize = 3);

/** @overload ComputeSubvoxelEigenPairs() */
EigenPairs ComputeSubvoxelEigenPairs(
    const Volume::Pointer& volume,
    const cv::Vec3d& index,
    int radius = 1,
    int kernelSize = 3);

/**
 * @brief Get an axis-aligned cuboid subvolume centered on a voxel
 * @param center Center position of the subvolume
 * @param rx Radius of the subvolume X-axis
 * @param ry Radius of the subvolume Y-axis
 * @param rz Radius of the subvolume Z-axis
 */
template <typename DType>
Tensor3D<DType> ComputeVoxelNeighbors(
    const Volume::Pointer& volume,
    const cv::Vec3i& center,
    int32_t rx,
    int32_t ry,
    int32_t rz)
{
    Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
    for (int32_t k = center(2) - rz, c = 0; k <= center(2) + rz; ++k, ++c) {
        for (int32_t j = center(1) - ry, b = 0; j <= center(1) + ry; ++j, ++b) {
            for (int32_t i = center(0) - rx, a = 0; i <= center(0) + rx;
                 ++i, ++a) {
                v(a, b, c) = DType(volume->intensityAt(i, j, k));
            }
        }
    }

    return v;
}

/**
 * @brief Get an axis-aligned cuboid subvolume centered on a subvoxel
 *
 * @copydetails getVoxelNeighbors()
 */
template <typename DType>
Tensor3D<DType> ComputeSubvoxelNeighbors(
    const Volume::Pointer& volume,
    const cv::Vec3d& center,
    int rx,
    int ry,
    int rz,
    const cv::Vec3d& xvec = {1, 0, 0},
    const cv::Vec3d& yvec = {0, 1, 0},
    const cv::Vec3d& zvec = {0, 0, 1})
{
    Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
    for (int c = 0; c < 2 * rz + 1; ++c) {
        for (int b = 0; b < 2 * ry + 1; ++b) {
            for (int a = 0; a < 2 * rx + 1; ++a) {
                auto xOffset = -rx + a;
                auto yOffset = -ry + b;
                auto zOffset = -rz + c;
                auto p = center + (xvec * xOffset) + (yvec * yOffset) +
                         (zvec * zOffset);
                v(a, b, c) = DType(volume->interpolateAt(p));
            }
        }
    }

    return v;
}
}  // namespace volcart
