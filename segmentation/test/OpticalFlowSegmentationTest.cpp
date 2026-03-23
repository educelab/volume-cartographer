#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/OpticalFlowSegmentation.hpp"

using namespace volcart::segmentation;

////////////////////////////////////////////////////////////////////////////////
// Setter/getter round-trips — no volume required

TEST(OpticalFlowSegmentationTest, SetGetStartZIndex)
{
    OpticalFlowSegmentation ofs;
    ofs.setStartZIndex(42);
    EXPECT_EQ(ofs.getStartZIndex(), 42);
}

TEST(OpticalFlowSegmentationTest, SetGetTargetZIndex)
{
    OpticalFlowSegmentation ofs;
    ofs.setTargetZIndex(100);
    EXPECT_EQ(ofs.getTargetZIndex(), 100);
}

TEST(OpticalFlowSegmentationTest, SetGetInterpolationWindow)
{
    OpticalFlowSegmentation ofs;
    ofs.setInterpolationWindow(15);
    EXPECT_EQ(ofs.getInterpolationWindow(), std::uint32_t{15});
}

TEST(OpticalFlowSegmentationTest, SetGetInterpolationDistance)
{
    OpticalFlowSegmentation ofs;
    ofs.setInterpolationDistance(50);
    EXPECT_EQ(ofs.getInterpolationDistance(), std::uint32_t{50});
}

TEST(OpticalFlowSegmentationTest, MaxThreadsRoundTrip)
{
    OpticalFlowSegmentation ofs;
    ofs.setMaxThreads(4);
    // resetMaxThreads() should not crash
    ofs.resetMaxThreads();
}

////////////////////////////////////////////////////////////////////////////////
// compute() smoke test using the Testing.volpkg fixture

class OpticalFlowSegmentationFix : public ::testing::Test
{
public:
    OpticalFlowSegmentationFix() = default;

    volcart::VolumePkg pkg_{"Testing.volpkg"};
    OpticalFlowSegmentation segmenter_;
};

TEST_F(OpticalFlowSegmentationFix, SmokeTest)
{
    // Seed chain from the shared test fixture (z = 1)
    auto pathSeed = pkg_.segmentation("starting-path")->getPointSet().getRow(0);

    segmenter_.setChain(pathSeed);
    segmenter_.setVolume(pkg_.volume());
    segmenter_.setStartZIndex(1);
    segmenter_.setTargetZIndex(5);
    segmenter_.setMaterialThickness(pkg_.materialThickness());
    // Disable interpolation against master cloud to avoid needing a full run
    segmenter_.setInterpolate(false);
    segmenter_.setVisualize(false);
    segmenter_.setDumpVis(false);

    OpticalFlowSegmentation::PointSet result;
    ASSERT_NO_THROW(result = segmenter_.compute());
    EXPECT_GT(result.size(), std::size_t{0});
}
