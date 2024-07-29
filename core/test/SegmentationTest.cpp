#include <gtest/gtest.h>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/Segmentation.hpp"

namespace fs = volcart::filesystem;
using namespace volcart;

const fs::path VPKG{"Testing.volpkg"};

TEST(Segmentation, EmptySegmentation)
{
    // Load segmentation
    const auto p = VPKG / "paths" / "empty";
    Segmentation::Pointer seg;
    EXPECT_NO_THROW(seg = Segmentation::New(p));

    // Check interface
    EXPECT_EQ(seg->name(), "empty");
    EXPECT_EQ(seg->id(), "empty");
    EXPECT_FALSE(seg->hasPointSet());
    EXPECT_FALSE(seg->hasVolumeID());
    Segmentation::PointSet ps;
    EXPECT_THROW(ps = seg->getPointSet(), std::runtime_error);
    Volume::Identifier vid;
    EXPECT_THROW(vid = seg->getVolumeID(), std::runtime_error);
}

TEST(Segmentation, Initialized)
{
    // Load segmentation
    const auto p = VPKG / "paths" / "starting-path";
    Segmentation::Pointer seg;
    EXPECT_NO_THROW(seg = Segmentation::New(p));

    // Check interface
    EXPECT_EQ(seg->name(), "starting-path");
    EXPECT_EQ(seg->id(), "starting-path");
    EXPECT_TRUE(seg->hasPointSet());
    EXPECT_FALSE(seg->hasVolumeID());
    Segmentation::PointSet ps;
    EXPECT_NO_THROW(ps = seg->getPointSet());
    EXPECT_EQ(ps.width(), 135);
    EXPECT_EQ(ps.height(), 1);
    Volume::Identifier vid;
    EXPECT_THROW(vid = seg->getVolumeID(), std::runtime_error);
}