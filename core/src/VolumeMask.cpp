#include "vc/core/types/VolumeMask.hpp"

using namespace volcart;

VolumeMask::VolumeMask(
    std::size_t width, std::size_t height, std::size_t numSlices)
    : sliceWidth_{width}, sliceHeight_{height}
{

    // sparse matrix length decided by number of slices
    states_ = Eigen::SparseMatrix<int, Eigen::RowMajor>(
        height * numSlices, sliceWidth_);
}

void VolumeMask::setVoxelState(const cv::Vec3i& xyz, State state)
{
    states_.coeffRef(xyz[2] * sliceHeight_ + xyz[1], xyz[0]) =
        static_cast<int>(state);
}

void VolumeMask::setSubvolumeState(
    const cv::Vec3i& origin, const cv::Vec3i& dims, State state)
{
    cv::Vec3i pos;
    for (int zOffset = 0; zOffset < dims[2]; zOffset++) {
        for (int yOffset = 0; yOffset < dims[1]; yOffset++) {
            for (int xOffset = 0; xOffset < dims[0]; xOffset++) {
                pos[0] = origin[0] + xOffset;
                pos[1] = origin[1] + yOffset;
                pos[2] = origin[2] + zOffset;
                setVoxelState(pos, state);
            }
        }
    }
}

VolumeMask::State VolumeMask::getVoxelState(const cv::Vec3i& xyz)
{
    return static_cast<State>(
        states_.coeffRef(xyz[2] * sliceHeight_ + xyz[1], xyz[0]));
}

VolumeMask::Subvolume VolumeMask::getSubvolumeState(
    const cv::Vec3i& origin, const cv::Vec3i& dims)
{
    Subvolume result(3, dims[2], dims[1], dims[0]);
    int u, v;
    for (int zOffset = 0; zOffset < dims[2]; zOffset++) {
        for (int yOffset = 0; yOffset < dims[1]; yOffset++) {
            for (int xOffset = 0; xOffset < dims[0]; xOffset++) {
                u = origin[0] + xOffset;
                v = (origin[2] * sliceHeight_) + (zOffset * sliceHeight_) +
                    (origin[1] + yOffset);
                result(zOffset, yOffset, xOffset) =
                    static_cast<State>(states_.coeffRef(v, u));
            }
        }
    }

    return result;
}
