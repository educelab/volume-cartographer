#ifndef VC_PREBUILT_LIBS
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE LocalResliceSegmentation

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>
#include "volumepkg.h"
#include "localResliceParticleSim/localResliceParticleSim.h"

using namespace volcart::segmentation;

// Main fixture containing the LocalResliceSegmentation object
struct LocalResliceSegmentationFix {
    LocalResliceSegmentationFix() : _pkg("Testing.volpkg"), _segmenter(_pkg)
    {
        std::cout << "init volpkg and segmentation object" << std::endl;
    }

    ~LocalResliceSegmentationFix()
    {
        std::cout << "cleaning up fixture" << std::endl;
    }

    VolumePkg _pkg;
    LocalResliceSegmentation _segmenter;
};

// Test for default segmentation
BOOST_FIXTURE_TEST_CASE(DefaultSegmentationTest, LocalResliceSegmentationFix)
{
    // Get the cloud to compare against
    const std::string groundTruthSeg("local-reslice-particle-sim");
    _pkg.setActiveSegmentation(groundTruthSeg);
    const auto groundTruthCloud = _pkg.openCloud();

    // Get the starting cloud to segment
    const std::string startingCloudSeg("starting-path");
    _pkg.setActiveSegmentation(startingCloudSeg);
    auto startingCloud = _pkg.openCloud();

    // Run segmentation
    // XXX These params are manually input now, later they will be dynamically
    // read from the parameters.json file in each segmentation directory
    int32_t startIndex = 1;
    int32_t endIndex = 182;
    int32_t numIters = 15;
    int32_t stepNumLayers = 1;
    double alpha = 1.0 / 3.0;
    double k1 = 0.5;
    double k2 = 0.5;
    double beta = 1.0 / 3.0;
    double delta = 1.0 / 3.0;
    int32_t peakDistanceWeight = 50;
    bool shouldIncludeMiddle = false;
    bool dumpVis = true;
    bool visualize = false;
    auto resultCloud = _segmenter.segmentPath(
        startingCloud, startIndex, endIndex, numIters, stepNumLayers, alpha, k1,
        k2, beta, delta, peakDistanceWeight, shouldIncludeMiddle, dumpVis,
        visualize);
    _pkg.saveCloud(resultCloud);
    std::cout << "size: " << groundTruthCloud->size()
              << ", width: " << groundTruthCloud->width
              << ", height: " << groundTruthCloud->height << std::endl;
    std::cout << "size: " << resultCloud.size()
              << ", width: " << resultCloud.width
              << ", height: " << resultCloud.height << std::endl;

    // First compare cloud sizes
    BOOST_REQUIRE_EQUAL(groundTruthCloud->size(), resultCloud.size());
    BOOST_REQUIRE_EQUAL(groundTruthCloud->width, resultCloud.width);
    BOOST_REQUIRE_EQUAL(groundTruthCloud->height, resultCloud.height);

    // Compare clouds, make sure each point is within a certain tolerance.
    // Currently set in this file, may be ste outside later on
    constexpr double percentDifferenceTolerance = 0.1;  // %
    for (size_t i = 0; i < groundTruthCloud->size(); ++i) {
        BOOST_CHECK_CLOSE(groundTruthCloud->points[i].x,
                          resultCloud.points[i].x, percentDifferenceTolerance);
        BOOST_CHECK_CLOSE(groundTruthCloud->points[i].y,
                          resultCloud.points[i].y, percentDifferenceTolerance);
        BOOST_CHECK_CLOSE(groundTruthCloud->points[i].z,
                          resultCloud.points[i].z, percentDifferenceTolerance);
    }
}
