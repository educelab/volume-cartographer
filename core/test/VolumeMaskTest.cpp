#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "vc/core/types/VolumeMask.hpp"

namespace vc = volcart;

using VolumeMask = vc::VolumeMask;
using State = vc::VolumeMask::State;

constexpr int XY_SIZE = 3000;
constexpr int Z_SIZE = 3000;

TEST(VolumeMask, BasicAssignment)
{
    // Constructor
    VolumeMask mask(5, 5, 5);

    // Assignment
    const cv::Vec3i voxel(1, 2, 3);
    mask.setVoxelState(voxel, State::Segmented);

    // Retrieval
    EXPECT_EQ(mask.getVoxelState(voxel), State::Segmented);

    // Assignment
    mask.setVoxelState(voxel, State::Unsegmented);

    // Retrieval
    EXPECT_EQ(mask.getVoxelState(voxel), State::Unsegmented);
}

TEST(VolumeMask, Performance_LargeSlice)
{
    // Constructor
    VolumeMask mask(XY_SIZE, XY_SIZE, Z_SIZE);

    // Assignment
    const cv::Vec3i voxel(XY_SIZE / 10, XY_SIZE / 10, Z_SIZE / 10);

    mask.setVoxelState(voxel, State::Segmented);

    // Retrieval
    EXPECT_EQ(mask.getVoxelState(voxel), State::Segmented);
}

TEST(VolumeMask, Performance_ThousandVoxelSubvolume)
{
    // Constructor
    VolumeMask mask(XY_SIZE, XY_SIZE, Z_SIZE);

    // Assignment
    cv::Vec3i origin(0, 0, 0);
    cv::Vec3i dims(10, 10, 10);
    mask.setSubvolumeState(origin, dims, State::Segmented);

    // Retrieval
    auto subvolume = mask.getSubvolumeState(origin, dims);
    const auto count = static_cast<std::size_t>(dims[0]) *
                       static_cast<std::size_t>(dims[1]) *
                       static_cast<std::size_t>(dims[2]);
    std::vector key(count, State::Segmented);

    // Retrieval
    EXPECT_THAT(subvolume.as_vector(), testing::ContainerEq(key));

    // Check unset values
    origin[2] = dims[2];
    subvolume = mask.getSubvolumeState(origin, dims);
    std::fill(key.begin(), key.end(), State::Unsegmented);

    // Retrieval
    EXPECT_THAT(subvolume.as_vector(), testing::ContainerEq(key));
}

TEST(VolumeMask, Performance_EightThousandVoxelSubvolume)
{
    // Constructor
    VolumeMask mask(XY_SIZE, XY_SIZE, Z_SIZE);

    // Assignment
    cv::Vec3i origin(0, 0, 0);
    cv::Vec3i dims(20, 20, 20);
    mask.setSubvolumeState(origin, dims, State::Segmented);

    // Retrieval
    auto subvolume = mask.getSubvolumeState(origin, dims);
    const auto count = static_cast<std::size_t>(dims[0]) *
                       static_cast<std::size_t>(dims[1]) *
                       static_cast<std::size_t>(dims[2]);
    std::vector key(count, State::Segmented);

    // Retrieval
    EXPECT_THAT(subvolume.as_vector(), testing::ContainerEq(key));

    // Check unset values
    origin[2] = dims[2];
    subvolume = mask.getSubvolumeState(origin, dims);
    std::fill(key.begin(), key.end(), State::Unsegmented);

    // Retrieval
    EXPECT_THAT(subvolume.as_vector(), testing::ContainerEq(key));
}