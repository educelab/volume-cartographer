#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "vc/core/types/VolumePkg.hpp"
#include "vc/segmentation/LocalResliceParticleSim.hpp"

using namespace volcart::segmentation;

struct PointXYZ {

    double x, y, z;

    explicit PointXYZ(const cv::Vec3d& p) : x(p[0]), y(p[1]), z(p[2]) {}
};

auto operator<<(std::ostream& s, PointXYZ p) -> std::ostream&;

inline auto NormL2(const PointXYZ p1, const PointXYZ p2) -> double
{
    return std::sqrt(
        (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
        (p1.z - p2.z) * (p1.z - p2.z));
}

// Main fixture containing the LocalResliceSegmentation object
class LocalResliceSegmentationFix : public ::testing::Test
{
public:
    LocalResliceSegmentationFix() = default;

    volcart::VolumePkg pkg_{"Testing.volpkg"};
    LocalResliceSegmentation segmenter_;
};

// Test for default segmentation
TEST_F(LocalResliceSegmentationFix, DefaultSegmentationTest)
{
    // Get the cloud to compare against
    auto groundTruthSeg = pkg_.segmentation("local-reslice-particle-sim");
    const auto groundTruthCloud = groundTruthSeg->getPointSet();

    // Get the starting cloud to segment
    auto pathSeed = pkg_.segmentation("starting-path")->getPointSet().getRow(0);

    // Run segmentation
    // XXX These params are manually input now, later they will be dynamically
    // read from the parameters.json file in each segmentation directory
    int endIndex = 182;
    int numIters = 15;
    int stepNumLayers = 1;
    double alpha = 1.0 / 3.0;
    double k1 = 0.5;
    double k2 = 0.5;
    double beta = 1.0 / 3.0;
    double delta = 1.0 / 3.0;
    int peakDistanceWeight = 50;
    bool shouldIncludeMiddle = false;
    bool dumpVis = false;
    bool visualize = false;

    segmenter_.setChain(pathSeed);
    segmenter_.setVolume(pkg_.volume());
    segmenter_.setTargetZIndex(endIndex);
    segmenter_.setStepSize(stepNumLayers);
    segmenter_.setOptimizationIterations(numIters);
    segmenter_.setAlpha(alpha);
    segmenter_.setK1(k1);
    segmenter_.setK2(k2);
    segmenter_.setBeta(beta);
    segmenter_.setDelta(delta);
    segmenter_.setMaterialThickness(pkg_.materialThickness());
    segmenter_.setDistanceWeightFactor(peakDistanceWeight);
    segmenter_.setConsiderPrevious(shouldIncludeMiddle);
    segmenter_.setVisualize(visualize);
    segmenter_.setDumpVis(dumpVis);
    auto resultCloud = segmenter_.compute();

    // Save the results
    auto testCloudSeg = pkg_.newSegmentation("lrps-test-results");
    testCloudSeg->setPointSet(resultCloud);

    // First compare cloud sizes
    ASSERT_EQ(groundTruthCloud.size(), resultCloud.size());
    ASSERT_EQ(groundTruthCloud.width(), resultCloud.width());
    ASSERT_EQ(groundTruthCloud.height(), resultCloud.height());

    // Compare clouds, make sure each point is within a certain tolerance.
    // Currently set in this file, may be set outside later on
    constexpr double voxelDiffTol = 10;  // %
    std::size_t diffCount = 0;
    for (std::size_t i = 0; i < groundTruthCloud.size(); ++i) {
        PointXYZ trueV(groundTruthCloud[i]);
        PointXYZ testV(resultCloud[i]);
        auto xdiff = std::abs(trueV.x - testV.x);
        auto ydiff = std::abs(trueV.y - testV.y);
        auto zdiff = std::abs(trueV.z - testV.z);
        auto normDiff = NormL2(trueV, testV);

        if (xdiff > voxelDiffTol || ydiff > voxelDiffTol ||
            zdiff > voxelDiffTol || normDiff > voxelDiffTol) {
            diffCount++;
        }

        EXPECT_PRED_FORMAT2(::testing::DoubleLE, normDiff, voxelDiffTol);
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, xdiff, voxelDiffTol);
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, ydiff, voxelDiffTol);
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, zdiff, voxelDiffTol);
    }

    // Check that the clouds never vary in point differences by 10%
    auto maxAllowedDiffCount =
        std::size_t(std::round(0.1 * groundTruthCloud.size()));
    std::cout << "# different points: " << diffCount
              << " (max allowed: " << maxAllowedDiffCount << ")" << '\n';
    EXPECT_TRUE(diffCount < maxAllowedDiffCount);
}

TEST_F(LocalResliceSegmentationFix, NoMemMapSegmentationTest)
{
    // Disable memory mapping
    pkg_.volume()->setMemoryMapSlices(false);

    // Get the cloud to compare against
    auto groundTruthSeg = pkg_.segmentation("local-reslice-particle-sim");
    const auto groundTruthCloud = groundTruthSeg->getPointSet();

    // Get the starting cloud to segment
    auto pathSeed = pkg_.segmentation("starting-path")->getPointSet().getRow(0);

    // Run segmentation
    // XXX These params are manually input now, later they will be dynamically
    // read from the parameters.json file in each segmentation directory
    int endIndex = 182;
    int numIters = 15;
    int stepNumLayers = 1;
    double alpha = 1.0 / 3.0;
    double k1 = 0.5;
    double k2 = 0.5;
    double beta = 1.0 / 3.0;
    double delta = 1.0 / 3.0;
    int peakDistanceWeight = 50;
    bool shouldIncludeMiddle = false;
    bool dumpVis = false;
    bool visualize = false;

    segmenter_.setChain(pathSeed);
    segmenter_.setVolume(pkg_.volume());
    segmenter_.setTargetZIndex(endIndex);
    segmenter_.setStepSize(stepNumLayers);
    segmenter_.setOptimizationIterations(numIters);
    segmenter_.setAlpha(alpha);
    segmenter_.setK1(k1);
    segmenter_.setK2(k2);
    segmenter_.setBeta(beta);
    segmenter_.setDelta(delta);
    segmenter_.setMaterialThickness(pkg_.materialThickness());
    segmenter_.setDistanceWeightFactor(peakDistanceWeight);
    segmenter_.setConsiderPrevious(shouldIncludeMiddle);
    segmenter_.setVisualize(visualize);
    segmenter_.setDumpVis(dumpVis);
    auto resultCloud = segmenter_.compute();

    // Save the results
    auto testCloudSeg = pkg_.newSegmentation("lrps-test-results");
    testCloudSeg->setPointSet(resultCloud);

    // First compare cloud sizes
    ASSERT_EQ(groundTruthCloud.size(), resultCloud.size());
    ASSERT_EQ(groundTruthCloud.width(), resultCloud.width());
    ASSERT_EQ(groundTruthCloud.height(), resultCloud.height());

    // Compare clouds, make sure each point is within a certain tolerance.
    // Currently set in this file, may be set outside later on
    constexpr double voxelDiffTol = 10;  // %
    std::size_t diffCount = 0;
    for (std::size_t i = 0; i < groundTruthCloud.size(); ++i) {
        PointXYZ trueV(groundTruthCloud[i]);
        PointXYZ testV(resultCloud[i]);
        auto xdiff = std::abs(trueV.x - testV.x);
        auto ydiff = std::abs(trueV.y - testV.y);
        auto zdiff = std::abs(trueV.z - testV.z);
        auto normDiff = NormL2(trueV, testV);

        if (xdiff > voxelDiffTol || ydiff > voxelDiffTol ||
            zdiff > voxelDiffTol || normDiff > voxelDiffTol) {
            diffCount++;
        }

        EXPECT_PRED_FORMAT2(::testing::DoubleLE, normDiff, voxelDiffTol);
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, xdiff, voxelDiffTol);
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, ydiff, voxelDiffTol);
        EXPECT_PRED_FORMAT2(::testing::DoubleLE, zdiff, voxelDiffTol);
    }

    // Check that the clouds never vary in point differences by 10%
    auto maxAllowedDiffCount =
        std::size_t(std::round(0.1 * groundTruthCloud.size()));
    std::cout << "# different points: " << diffCount
              << " (max allowed: " << maxAllowedDiffCount << ")" << '\n';
    EXPECT_TRUE(diffCount < maxAllowedDiffCount);
}

auto operator<<(std::ostream& s, PointXYZ p) -> std::ostream&
{
    return s << "[" << p.x << ", " << p.y << ", " << p.z << "]";
}